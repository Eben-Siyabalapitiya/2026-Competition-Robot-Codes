"""
MiniBot vision + control for the RoboHax Sector Siege Autonomous Control Challenge.

Camera watches the field from the driver station, finds the AprilTag on top of
the robot, works out where the robot is in field coordinates, and sends that
pose to the robot over ESP-NOW through a USB dongle. You click a destination or
press a number for a platform; the robot plans and drives there itself.

Only high-level intent is ever sent - a destination or a target id, never
steering. That is what keeps this inside "Augmented Control" and out of direct
teleoperation.

Field frame: origin at the blue end corner, +X toward the orange end,
+Y across the field, millimetres. 1829 x 1219 mm (72 x 48 in).

Keys
  SPACE   enable (the one start action on the field signal)
  D       disable / stop everything
  C       cancel the current goal, stay enabled
  1..6    drive to a platform (see TARGETS)
  R       redo calibration
  Z       zero the heading - robot must be facing +X
  Q       quit

Mouse
  left click on the video   drive to that spot on the field
"""

import argparse
import math
import struct
import sys
import time

import cv2
import numpy as np
import serial
from pupil_apriltags import Detector

FIELD_X_MM = 1829
FIELD_Y_MM = 1219

LINK_MAGIC = 0x5A
LINK_VERSION = 1

CMD_NONE = 0
CMD_GOTO_XY = 1
CMD_GOTO_TGT = 2
CMD_CANCEL = 3
CMD_ENABLE = 4
CMD_DISABLE = 5

PACKET_FMT = "<BBHhhhBBhhBB"
assert struct.calcsize(PACKET_FMT) == 18

TARGETS = {
    ord("1"): (1, "corner blue A"),
    ord("2"): (2, "corner blue B"),
    ord("3"): (3, "corner orange A"),
    ord("4"): (4, "corner orange B"),
    ord("5"): (5, "centre blue"),
    ord("6"): (6, "centre orange"),
}

# platform centres - VERIFY against the arena CAD.
CALIB_POINTS = [
    (305, 305),
    (305, 914),
    (1524, 914),
    (1524, 305),
]

def checksum(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c

class Link:
    """Serial link to the ESP-NOW dongle."""

    def __init__(self, port, baud=921600):
        self.seq = 0
        self.ser = None
        if port:
            self.ser = serial.Serial(port, baud, timeout=0)
            time.sleep(2.0)
            print(f"dongle open on {port}")
        else:
            print("no --port given, running vision only (nothing will be sent)")

    def send(self, pose, pose_valid, cmd=CMD_NONE, gx=0, gy=0, target=0):
        if self.ser is None:
            return
        self.seq = (self.seq + 1) & 0xFFFF
        x, y, heading = pose
        body = struct.pack(
            PACKET_FMT,
            LINK_MAGIC,
            LINK_VERSION,
            self.seq,
            int(x),
            int(y),
            int(heading * 100.0),
            1 if pose_valid else 0,
            cmd,
            int(gx),
            int(gy),
            target,
            0,
        )
        body = body[:-1] + bytes([checksum(body[:-1])])
        self.ser.write(b"\xAB\xCD" + body)

    def close(self):
        if self.ser:
            self.ser.close()

class FieldMap:
    """Image <-> field homography, solved from the tag at known field points."""

    def __init__(self):
        self.H = None
        self.H_inv = None
        self.heading_offset = 0.0

    @property
    def ready(self):
        return self.H is not None

    def solve(self, image_pts, field_pts):
        src = np.array(image_pts, dtype=np.float32)
        dst = np.array(field_pts, dtype=np.float32)
        H, _ = cv2.findHomography(src, dst, 0)
        if H is None:
            return False
        self.H = H
        self.H_inv = np.linalg.inv(H)
        return True

    def to_field(self, pt):
        p = np.array([pt[0], pt[1], 1.0])
        q = self.H @ p
        return (q[0] / q[2], q[1] / q[2])

    def to_image(self, pt):
        p = np.array([pt[0], pt[1], 1.0])
        q = self.H_inv @ p
        return (int(q[0] / q[2]), int(q[1] / q[2]))

def tag_pose(det, fmap):
    """Field pose of a detected tag: (x_mm, y_mm, heading_deg)."""
    cx, cy = det.center
    c = det.corners
    front_img = ((c[1][0] + c[2][0]) / 2.0, (c[1][1] + c[2][1]) / 2.0)

    x, y = fmap.to_field((cx, cy))
    fx, fy = fmap.to_field(front_img)
    heading = math.degrees(math.atan2(fy - y, fx - x)) - fmap.heading_offset
    while heading > 180.0:
        heading -= 360.0
    while heading < -180.0:
        heading += 360.0
    return x, y, heading

def draw_overlay(frame, fmap, pose, enabled, status):
    if fmap.ready:
        corners = [(0, 0), (FIELD_X_MM, 0), (FIELD_X_MM, FIELD_Y_MM), (0, FIELD_Y_MM)]
        pts = np.array([fmap.to_image(p) for p in corners], dtype=np.int32)
        cv2.polylines(frame, [pts], True, (0, 220, 255), 2)
        for cxy in [(610, 610), (1219, 610)]:
            cv2.circle(frame, fmap.to_image(cxy), 6, (255, 120, 0), -1)
        for t in CALIB_POINTS:
            cv2.circle(frame, fmap.to_image(t), 5, (0, 255, 0), 1)

    bar = "ENABLED" if enabled else "disabled"
    colour = (0, 255, 0) if enabled else (0, 0, 255)
    cv2.putText(frame, bar, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)

    if pose:
        txt = f"x={pose[0]:.0f} y={pose[1]:.0f} hdg={pose[2]:.1f}"
    else:
        txt = "NO TAG"
    cv2.putText(frame, txt, (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(frame, status, (12, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

def run_calibration(cap, detector, fmap, tag_id):
    """Place the robot on each known point in turn and press SPACE."""
    captured = []
    idx = 0
    print("\ncalibration: drive the robot onto each green point, press SPACE")

    while idx < len(CALIB_POINTS):
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dets = [d for d in detector.detect(gray) if tag_id is None or d.tag_id == tag_id]

        for d in dets:
            c = np.array(d.corners, dtype=np.int32)
            cv2.polylines(frame, [c], True, (0, 255, 0), 2)
            cv2.circle(frame, (int(d.center[0]), int(d.center[1])), 4, (0, 0, 255), -1)

        target = CALIB_POINTS[idx]
        cv2.putText(
            frame,
            f"CALIBRATE {idx + 1}/4  put robot at field {target}  SPACE=capture  Q=quit",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
        cv2.imshow("minibot", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            return False
        if key == ord(" "):
            if not dets:
                print("  no tag visible, not captured")
                continue
            captured.append(dets[0].center)
            print(f"  point {idx + 1} captured at image {dets[0].center}")
            idx += 1

    if not fmap.solve(captured, CALIB_POINTS):
        print("homography failed, try again with the points further apart")
        return False

    print("calibration done")
    return True

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--port", default=None, help="dongle COM port, e.g. COM5")
    ap.add_argument("--tag-id", type=int, default=None, help="only track this tag id")
    ap.add_argument("--family", default="tag36h11")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"cannot open camera {args.camera}")
        return 1

    detector = Detector(families=args.family, nthreads=4, quad_decimate=1.0)
    fmap = FieldMap()
    link = Link(args.port)

    click_goal = {"pt": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and fmap.ready:
            click_goal["pt"] = fmap.to_field((x, y))

    cv2.namedWindow("minibot")
    cv2.setMouseCallback("minibot", on_mouse)

    if not run_calibration(cap, detector, fmap, args.tag_id):
        cap.release()
        cv2.destroyAllWindows()
        return 1

    enabled = False
    status = "calibrated - SPACE to enable"
    last_send = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            dets = [d for d in detector.detect(gray) if args.tag_id is None or d.tag_id == args.tag_id]

            pose = None
            if dets:
                d = dets[0]
                pose = tag_pose(d, fmap)
                c = np.array(d.corners, dtype=np.int32)
                cv2.polylines(frame, [c], True, (0, 255, 0), 2)

            cmd, gx, gy, target = CMD_NONE, 0, 0, 0

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):
                cmd, enabled = CMD_ENABLE, True
                status = "ENABLE sent"
            elif key == ord("d"):
                cmd, enabled = CMD_DISABLE, False
                status = "DISABLE sent"
            elif key == ord("c"):
                cmd = CMD_CANCEL
                status = "cancel sent"
            elif key == ord("r"):
                if not run_calibration(cap, detector, fmap, args.tag_id):
                    break
                status = "recalibrated"
            elif key == ord("z"):
                if pose:
                    fmap.heading_offset += pose[2]
                    status = f"heading zeroed (offset {fmap.heading_offset:.1f})"
            elif key in TARGETS:
                target, name = TARGETS[key]
                cmd = CMD_GOTO_TGT
                status = f"goto {name}"

            if click_goal["pt"] is not None:
                gx, gy = click_goal["pt"]
                click_goal["pt"] = None
                cmd = CMD_GOTO_XY
                status = f"goto {gx:.0f},{gy:.0f}"

            now = time.time()
            if cmd != CMD_NONE or (now - last_send) > 0.02:
                link.send(pose or (0, 0, 0), pose is not None, cmd, gx, gy, target)
                last_send = now

            draw_overlay(frame, fmap, pose, enabled, status)
            cv2.imshow("minibot", frame)
    finally:
        link.send((0, 0, 0), False, CMD_DISABLE)
        link.close()
        cap.release()
        cv2.destroyAllWindows()

    return 0

if __name__ == "__main__":
    sys.exit(main())
