"""
Browser control panel for the MiniBot.

Runs the camera, finds the AprilTag, streams the annotated video to a webpage,
and sends pose + high-level commands to the robot. Everything you need during a
run is on the page: calibration, enable/disable, platform buttons, and
click-anywhere-to-drive.

The pose stream runs in its own thread and goes out continuously no matter what
the browser is doing, so a slow page never slows the robot's control loop. The
page is a viewer and an input device, nothing more.

Start it:
    python web_ui.py
Then open http://localhost:8000
"""

import argparse
import threading
import time
from collections import deque

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template, request
from pupil_apriltags import Detector

from minibot_vision import (
    CALIB_POINTS,
    CMD_CANCEL,
    CMD_DISABLE,
    CMD_ENABLE,
    CMD_GOTO_TGT,
    CMD_GOTO_XY,
    CMD_NONE,
    CMD_MANUAL,
    CMD_PATH_ADD,
    CMD_PATH_CLEAR,
    CMD_PATH_RUN,
    PATH_MAX,
    FIELD_X_MM,
    FIELD_Y_MM,
    FieldMap,
    Link,
    tag_pose,
)

from bluetooth_link import BluetoothLink

app = Flask(__name__)
app.config["TEMPLATES_AUTO_RELOAD"] = True

CALIB_NAMES = ["Blue A", "Blue B", "Orange B", "Orange A"]

PATH_EPSILON_MM = 55.0
LINK_HZ = 20
CMD_REPEATS = 1

def thin_path(pts):
    if len(pts) < 3:
        return pts
    ax, ay = pts[0]
    bx, by = pts[-1]
    dx, dy = bx - ax, by - ay
    span = (dx * dx + dy * dy) ** 0.5
    worst, worst_i = -1.0, 0
    for i in range(1, len(pts) - 1):
        px, py = pts[i]
        if span < 1e-6:
            d = ((px - ax) ** 2 + (py - ay) ** 2) ** 0.5
        else:
            d = abs(dy * px - dx * py + bx * ay - by * ax) / span
        if d > worst:
            worst, worst_i = d, i
    if worst <= PATH_EPSILON_MM:
        return [pts[0], pts[-1]]
    return thin_path(pts[: worst_i + 1])[:-1] + thin_path(pts[worst_i:])

TARGET_POINTS = {
    1: ((305, 305), "corner blue A"),
    2: ((305, 914), "corner blue B"),
    3: ((1524, 305), "corner orange A"),
    4: ((1524, 914), "corner orange B"),
    5: ((610, 610), "centre blue"),
    6: ((1219, 610), "centre orange"),
}

GOAL_REACHED_MM = 60.0

class Controller:
    def __init__(self, camera, port, tag_id, family, width, height, exposure=-5, tag_mm=127.0, robot_port=None):
        self.camera_index = camera
        self.width = width
        self.height = height
        self.exposure = exposure
        self.tag_mm = tag_mm
        self.cap = self.open_camera()
        self.detector = Detector(families=family, nthreads=4, quad_decimate=1.0)
        self.fmap = FieldMap()
        self.link = BluetoothLink(port)
        self.tag_id = tag_id

        self.lock = threading.Lock()
        self.frame = None
        self.frame_time = 0.0
        self.pose = None
        self.pose_time = 0.0
        self.tag_center = None
        self.tag_heading = None
        self.tag_px = None
        self.enabled = False
        self.status = "not calibrated"
        self.calib_shots = [None] * len(CALIB_POINTS)

        self.manual_token = None
        self.manual_deadline = 0.0
        self.manual_values = (0, 0)
        self.manual_retired = deque(maxlen=128)
        self.pending = deque()
        self.path = []
        self.goal = None
        self.goal_label = ""
        self.running = True

        threading.Thread(target=self.worker, daemon=True).start()
        threading.Thread(target=self.link_worker, daemon=True).start()

    def link_worker(self):
        """Send camera pose and commands over Bluetooth independently of video."""
        while self.running:
            self.link.poll()
            with self.lock:
                pose = self.pose if time.monotonic() - self.pose_time < 0.25 else None
                self.enabled = self.link.enabled
                item = self.next_command_locked()
                cmd, gx, gy, target = item if item else (CMD_NONE, 0, 0, 0)
                try:
                    self.link.send(pose or (0, 0, 0), pose is not None, cmd, gx, gy, target)
                except Exception as e:
                    print("link write failed:", e)
            time.sleep(1.0 / LINK_HZ)

    def retired_manual_locked(self):
        if not hasattr(self, 'manual_retired'):
            self.manual_retired = deque(maxlen=128)
        return self.manual_retired

    def end_manual_locked(self, cancel=True):
        token = getattr(self, 'manual_token', None)
        if token:
            self.retired_manual_locked().append(token)
            self.manual_token = None
            if cancel:
                self.pending.clear()
                self.pending.appendleft((CMD_CANCEL, 0, 0, 0))
        return token

    def next_command_locked(self):
        if getattr(self, 'manual_token', None) and time.monotonic() >= self.manual_deadline:
            self.end_manual_locked()
        if self.pending:
            return self.pending.popleft()
        if getattr(self, 'manual_token', None):
            forward, turn = self.manual_values
            return (CMD_MANUAL, forward, turn, 0)
        return None

    def open_camera(self):
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
        cap.set(cv2.CAP_PROP_SATURATION, 64)
        if self.exposure is not None:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        return cap

    def worker(self):
        black_frames = 0
        last_black_restart = 0.0
        last_send = 0.0
        misses = 0
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                misses += 1
                if misses >= 30:
                    print("camera stalled, reopening")
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    time.sleep(0.5)
                    self.cap = self.open_camera()
                    misses = 0
                time.sleep(0.01)
                continue
            misses = 0

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if float(gray.mean()) < 1.0:
                black_frames += 1
                with self.lock:
                    self.pose = None
                    self.tag_center = None
                    self.pose_time = 0.0
                if black_frames >= 10 and time.monotonic() - last_black_restart >= 5.0:
                    print("USB camera returned black frames; reopening", flush=True)
                    self.cap.release()
                    time.sleep(0.3)
                    self.cap = self.open_camera()
                    last_black_restart = time.monotonic()
                    black_frames = 0
                continue
            black_frames = 0
            dets = [
                d for d in self.detector.detect(gray)
                if self.tag_id is None or d.tag_id == self.tag_id
            ]

            pose = None
            centre = None
            tag_px = None
            tag_heading = None
            if dets:
                d = dets[0]
                centre = (float(d.center[0]), float(d.center[1]))
                cs = d.corners
                front = (cs[1] + cs[2]) / 2.0
                tag_heading = float(np.degrees(np.arctan2(-(front[1] - centre[1]), front[0] - centre[0])))
                tag_px = sum(
                    float(np.hypot(cs[k][0] - cs[(k + 1) % 4][0], cs[k][1] - cs[(k + 1) % 4][1]))
                    for k in range(4)
                ) / 4.0
                cv2.polylines(frame, [np.array(d.corners, dtype=np.int32)], True, (0, 255, 0), 2)
                cv2.circle(frame, (int(centre[0]), int(centre[1])), 5, (0, 0, 255), -1)
                if self.fmap.ready:
                    pose = tag_pose(d, self.fmap)

            if self.fmap.ready:
                self.draw_field(frame)
                self.draw_path(frame)
                self.draw_goal(frame, pose)

            with self.lock:
                self.pose = pose
                self.pose_time = time.monotonic()
                self.tag_center = centre
                self.tag_heading = tag_heading
                self.tag_px = tag_px
                self.frame = frame
                self.frame_time = time.monotonic()


    def draw_field(self, frame):
        corners = [(0, 0), (FIELD_X_MM, 0), (FIELD_X_MM, FIELD_Y_MM), (0, FIELD_Y_MM)]
        pts = np.array([self.fmap.to_image(p) for p in corners], dtype=np.int32)
        cv2.polylines(frame, [pts], True, (0, 220, 255), 2)
        for cxy in [(610, 610), (1219, 610)]:
            cv2.circle(frame, self.fmap.to_image(cxy), 8, (255, 120, 0), 2)
        for t in CALIB_POINTS:
            cv2.circle(frame, self.fmap.to_image(t), 5, (0, 255, 0), 1)

    def draw_path(self, frame):
        with self.lock:
            pts = list(self.path)
        if len(pts) < 2:
            return
        img = [self.fmap.to_image(p) for p in pts]
        for a, b in zip(img, img[1:]):
            cv2.line(frame, a, b, (255, 170, 60), 2, cv2.LINE_AA)
        for i, p in enumerate(img):
            cv2.circle(frame, p, 5, (255, 170, 60), -1, cv2.LINE_AA)
            cv2.circle(frame, p, 5, (20, 20, 20), 1, cv2.LINE_AA)
            cv2.putText(frame, str(i + 1), (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 170, 60), 1, cv2.LINE_AA)

    def draw_goal(self, frame, pose):
        with self.lock:
            goal = self.goal
            label = self.goal_label
        if goal is None:
            return

        reached = False
        if pose:
            dx = goal[0] - pose[0]
            dy = goal[1] - pose[1]
            reached = (dx * dx + dy * dy) ** 0.5 < GOAL_REACHED_MM

        colour = (80, 230, 80) if reached else (40, 190, 255)
        gx, gy = self.fmap.to_image(goal)

        if pose:
            rx, ry = self.fmap.to_image((pose[0], pose[1]))
            cv2.line(frame, (rx, ry), (gx, gy), colour, 1, cv2.LINE_AA)

        cv2.circle(frame, (gx, gy), 16, colour, 2, cv2.LINE_AA)
        cv2.circle(frame, (gx, gy), 4, colour, -1, cv2.LINE_AA)
        cv2.line(frame, (gx - 24, gy), (gx - 20, gy), colour, 2)
        cv2.line(frame, (gx + 20, gy), (gx + 24, gy), colour, 2)
        cv2.line(frame, (gx, gy - 24), (gx, gy - 20), colour, 2)
        cv2.line(frame, (gx, gy + 20), (gx, gy + 24), colour, 2)

        text = label + (" - reached" if reached else "")
        cv2.putText(frame, text, (gx + 22, gy - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, (gx + 22, gy - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)

    def set_goal(self, xy, label):
        with self.lock:
            self.goal = xy
            self.goal_label = label

    def clear_goal(self):
        with self.lock:
            self.goal = None
            self.goal_label = ""

    def queue(self, cmd, gx=0, gy=0, target=0):
        with self.lock:
            self.end_manual_locked()
            if cmd != CMD_DISABLE and not self.link.connected:
                self.status = "Connect Bluetooth first"
                return
            for _ in range(CMD_REPEATS):
                self.pending.append((cmd, gx, gy, target))

    def set_path(self, pts):
        with self.lock:
            self.end_manual_locked()
            self.path = list(pts)
            self.pending = deque(item for item in self.pending
                                 if item[0] in (CMD_ENABLE, CMD_DISABLE))
            if self.link.connected:
                self.pending.append((CMD_CANCEL, 0, 0, 0))
                self.pending.append((CMD_PATH_CLEAR, 0, 0, 0))
                self.pending.extend((CMD_PATH_ADD, px, py, i)
                                    for i, (px, py) in enumerate(pts))

    def clear_path(self):
        with self.lock:
            self.end_manual_locked()
            self.path = []
            self.pending = deque(item for item in self.pending
                                 if item[0] == CMD_DISABLE)
            self.pending.append((CMD_CANCEL, 0, 0, 0))
            self.pending.append((CMD_PATH_CLEAR, 0, 0, 0))

    def set_calib(self, index, point):
        if not (0 <= index < len(CALIB_POINTS)):
            return False, "no such calibration point"
        self.calib_shots[index] = (float(point[0]), float(point[1]))
        done = [p for p in self.calib_shots if p is not None]
        if len(done) < len(CALIB_POINTS):
            return True, f"{CALIB_NAMES[index]} set, {len(done)}/{len(CALIB_POINTS)} placed"
        if self.fmap.solve(self.calib_shots, CALIB_POINTS):
            self.status = "calibrated"
            return True, "calibrated"
        return False, "those four points do not form a valid field, move one"

    def quick_calibrate(self):
        with self.lock:
            centre = self.tag_center
            px = self.tag_px
        if centre is None or not px:
            return False, "no tag visible - point the camera at the tag first"
        mm_per_px = self.tag_mm / px
        s = mm_per_px
        tx = (FIELD_X_MM / 2.0) - s * centre[0]
        ty = (FIELD_Y_MM / 2.0) + s * centre[1]
        H = np.array([[s, 0.0, tx], [0.0, -s, ty], [0.0, 0.0, 1.0]])
        self.fmap.H = H
        self.fmap.H_inv = np.linalg.inv(H)
        self.calib_shots = [None] * len(CALIB_POINTS)
        self.status = "quick calibrated from the tag"
        return True, f"scale set from the tag: {mm_per_px:.2f} mm per pixel"

    def clear_calib_point(self, index):
        if 0 <= index < len(CALIB_POINTS):
            self.calib_shots[index] = None
            self.fmap.H = None
            self.fmap.H_inv = None
            self.status = "not calibrated"
            return True, f"{CALIB_NAMES[index]} cleared"
        return False, "no such calibration point"

    def reset_calib(self):
        self.calib_shots = [None] * len(CALIB_POINTS)
        self.fmap.H = None
        self.fmap.H_inv = None
        self.status = "not calibrated"

    def zero_heading(self, front=None):
        if self.link.enabled:
            return False, "Stop the robot before zeroing the tag"
        with self.lock:
            centre = self.tag_center
            px = self.tag_px
            heading = self.tag_heading
            fresh = time.monotonic() - self.pose_time < 0.25
            if centre is None or not px or heading is None or not fresh:
                return False, "Keep the tag visible, then try Zero tag again"
            offset = getattr(self, 'front_offset', None)
            if front and "x" in front and "y" in front:
                try:
                    dx = float(front["x"]) - centre[0]
                    dy = -(float(front["y"]) - centre[1])
                except (TypeError, ValueError):
                    return False, "Choose a valid point ahead of the robot's physical front"
                if not np.isfinite(dx) or not np.isfinite(dy) or np.hypot(dx, dy) < 30:
                    return False, "Click farther ahead of the robot's physical front"
                offset = heading - float(np.degrees(np.arctan2(dy, dx)))
            elif offset is None:
                return False, "First setup: click ahead of the robot's physical front"
            scale = self.tag_mm / px
            angle = np.radians(heading - offset)
            co, si = np.cos(angle), np.sin(angle)
            linear = scale * np.array([[co, -si], [-si, -co]])
            origin = np.array([FIELD_X_MM / 2.0, FIELD_Y_MM / 2.0])
            translation = origin - linear @ np.array(centre)
            H = np.eye(3)
            H[:2, :2] = linear
            H[:2, 2] = translation
            self.fmap.H = H
            self.fmap.H_inv = np.linalg.inv(H)
            self.fmap.heading_offset = offset
            self.front_offset = offset
            self.pose = (FIELD_X_MM / 2.0, FIELD_Y_MM / 2.0, 0.0)
            self.path = []
            self.goal = None
            self.goal_label = ""
            self.end_manual_locked(cancel=False)
            self.pending.clear()
            if self.link.connected:
                self.pending.append((CMD_PATH_CLEAR, 0, 0, 0))
        return True, "Heading zeroed to 0 degrees. Draw ahead of the robot to drive forward."

    def jpeg(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes() if ok else None

ctl: Controller = None

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/frame.jpg")
def frame_jpeg():
    buf = ctl.jpeg()
    if buf is None:
        return Response(status=503, headers={"Cache-Control": "no-store"})
    return Response(buf, mimetype="image/jpeg", headers={"Cache-Control": "no-store, max-age=0",
        "X-Frame-Age-Ms": str(int((time.monotonic() - getattr(ctl, "frame_time", 0.0)) * 1000))})


@app.route("/stream")
def stream():
    def gen():
        while True:
            buf = ctl.jpeg()
            if buf is not None:
                yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + buf + b"\r\n"
            time.sleep(0.033)

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=f")


@app.route("/api/bluetooth/ports")
def bluetooth_ports():
    from serial.tools import list_ports
    return jsonify([{"port": p.device, "name": p.description} for p in list_ports.comports()])

@app.route("/api/bluetooth/connect", methods=["POST"])
def bluetooth_connect():
    with ctl.lock:
        ctl.end_manual_locked(cancel=False)
        ctl.pending.clear()
        ctl.enabled = False
        ok, message = ctl.link.connect(str(request.json.get("port", "")))
    return jsonify({"message": message}), 200 if ok else 400

@app.route("/api/bluetooth/disconnect", methods=["POST"])
def bluetooth_disconnect():
    with ctl.lock:
        ctl.end_manual_locked(cancel=False)
        ctl.pending.clear()
        ctl.enabled = False
        ctl.link.close()
    return jsonify({"message": "Bluetooth disconnected; robot link-loss stop applies"})

@app.route("/api/state")
def state():
    with ctl.lock:
        pose = ctl.pose
        goal = ctl.goal
        goal_label = ctl.goal_label
        tag = ctl.tag_center
        frame = ctl.frame
        path = [list(p) for p in ctl.path]
    size = [int(frame.shape[1]), int(frame.shape[0])] if frame is not None else None
    return jsonify({
        "tag": list(tag) if tag else None,
        "frame_size": size,
        "pose": list(pose) if pose else None,
        "goal": list(goal) if goal else None,
        "goal_label": goal_label,
        "enabled": ctl.link.enabled,
        "remaining_ms": ctl.link.telemetry.get("remaining_ms", 0),
        "bluetooth": ctl.link.status(),
        "status": ctl.status,
        "calibrated": ctl.fmap.ready,
        "calib_points": len([p for p in ctl.calib_shots if p is not None]),
        "calib": [
            {"name": CALIB_NAMES[i], "field": list(CALIB_POINTS[i]),
             "image": list(ctl.calib_shots[i]) if ctl.calib_shots[i] else None}
            for i in range(len(CALIB_POINTS))
        ],
        "path": path,
        "field": [FIELD_X_MM, FIELD_Y_MM],
        "link_open": ctl.link.ser is not None,
        "link_sent": ctl.link.sent,
    })


@app.route("/api/manual", methods=["POST"])
def manual():
    data = request.get_json(silent=True) or {}
    token = data.get("token")
    direction = data.get("direction")
    if not isinstance(token, str) or not 1 <= len(token) <= 80:
        return jsonify({"message": "Invalid hold token"}), 400
    with ctl.lock:
        retired = ctl.retired_manual_locked()
        if direction == "stop":
            if getattr(ctl, 'manual_token', None) == token:
                ctl.end_manual_locked()
            elif token not in retired:
                retired.append(token)
            return jsonify({"ok": True})
        directions = {"forward": (22, 0), "backward": (-22, 0), "left": (0, 20), "right": (0, -20)}
        if direction not in directions:
            return jsonify({"message": "Invalid direction"}), 400
        try:
            age = time.time() * 1000 - float(data.get("sent_ms", 0))
        except (TypeError, ValueError):
            age = float('inf')
        if not np.isfinite(age) or age > 250 or age < -2000 or token in retired:
            return jsonify({"message": "Hold expired"}), 409
        if not ctl.link.enabled or ctl.link.telemetry.get('firmware') != 'bt-nav-8-manual':
            return jsonify({"message": "Enable first"}), 409
        active = getattr(ctl, 'manual_token', None)
        if active and time.monotonic() >= ctl.manual_deadline:
            ctl.end_manual_locked()
            if token in retired:
                return jsonify({"message": "Hold expired"}), 409
            active = None
        if active and active != token:
            return jsonify({"message": "Release the other button"}), 409
        if any(item[0] == CMD_DISABLE for item in ctl.pending):
            return jsonify({"message": "Robot stopping"}), 409
        if active is None:
            ctl.pending.clear()
            ctl.goal = None
            ctl.goal_label = ""
        ctl.manual_token = token
        ctl.manual_values = directions[direction]
        ctl.manual_deadline = time.monotonic() + 0.20
        ctl.status = "Manual"
        return jsonify({"ok": True})


@app.route("/api/enable", methods=["POST"])
def enable():
    if not ctl.link.connected:
        return jsonify({"message": "Connect Bluetooth and wait for robot status first"}), 409
    if ctl.link.telemetry.get('firmware') != 'bt-nav-8-manual':
        return jsonify({'message': 'Upload the corrected navigation firmware first'}), 409
    ctl.status = "Enable requested; waiting for robot"
    ctl.queue(CMD_ENABLE)
    return jsonify({"message": "Enable requested"})

@app.route("/api/disable", methods=["POST"])
def disable():
    ctl.enabled = False
    ctl.status = "DISABLE sent"
    ctl.clear_goal()
    with ctl.lock:
        ctl.end_manual_locked(cancel=False)
        ctl.pending.clear()
        ctl.pending.appendleft((CMD_DISABLE, 0, 0, 0))
    return jsonify({"message": "disabled"})

@app.route("/api/cancel", methods=["POST"])
def cancel():
    with ctl.lock:
        ctl.end_manual_locked(cancel=False)
        ctl.pending.clear()
    ctl.status = "goal cancelled"
    ctl.clear_goal()
    ctl.queue(CMD_CANCEL)
    return jsonify({"message": "cancelled"})

@app.route("/api/target", methods=["POST"])
def target():
    tid = int(request.json.get("id", 0))
    if not ctl.fmap.ready:
        return jsonify({"message": "calibrate first"})
    if tid not in TARGET_POINTS:
        return jsonify({"message": f"unknown target {tid}"})
    xy, name = TARGET_POINTS[tid]
    ctl.set_goal(xy, name)
    ctl.status = f"driving to {name}"
    ctl.queue(CMD_GOTO_TGT, target=tid)
    return jsonify({"message": f"selected {name}"})

@app.route("/api/click", methods=["POST"])
def click():
    if not ctl.fmap.ready:
        return jsonify({"message": "calibrate first"})
    x = float(request.json["x"])
    y = float(request.json["y"])
    fx, fy = ctl.fmap.to_field((x, y))
    if not (0 <= fx <= FIELD_X_MM and 0 <= fy <= FIELD_Y_MM):
        return jsonify({"message": "that point is off the field"})
    ctl.set_goal((fx, fy), f"{fx:.0f},{fy:.0f}")
    ctl.status = f"driving to {fx:.0f},{fy:.0f}"
    ctl.queue(CMD_GOTO_XY, gx=fx, gy=fy)
    return jsonify({"message": f"going to {fx:.0f},{fy:.0f}"})

@app.route("/api/path", methods=["POST"])
def path_set():
    if not ctl.fmap.ready:
        return jsonify({"message": "calibrate first"})
    raw = request.json.get("points", [])
    if len(raw) < 1:
        return jsonify({"message": "draw a line first"})
    pts = []
    for p in raw:
        fx, fy = ctl.fmap.to_field((float(p["x"]), float(p["y"])))
        if not (0 <= fx <= FIELD_X_MM and 0 <= fy <= FIELD_Y_MM):
            return jsonify({"message": "part of that path is off the field"})
        pts.append((int(round(fx)), int(round(fy))))
    if len(pts) == 1:
        with ctl.lock:
            current_pose = ctl.pose
        if current_pose and np.hypot(pts[0][0] - current_pose[0], pts[0][1] - current_pose[1]) <= 80:
            return jsonify({"message": "That point is inside the 8 cm stopping zone. Choose a farther point."}), 409
    pts = thin_path(pts)
    if len(pts) > PATH_MAX:
        return jsonify({"message": f"path too complex, {len(pts)} points but the limit is {PATH_MAX}"})
    ctl.set_path(pts)
    ctl.status = f"path loaded, {len(pts)} point" + ("s" if len(pts) != 1 else "") + ""
    return jsonify({"message": f"path loaded, {len(pts)} point" + ("s" if len(pts) != 1 else "") + "", "count": len(pts)})

@app.route("/api/path/run", methods=["POST"])
def path_run():
    with ctl.lock:
        n = len(ctl.path)
        last = ctl.path[-1] if ctl.path else None
    if not ctl.link.enabled:
        return jsonify({"message": "Enable the robot before Run line"}), 409
    if n == 0:
        return jsonify({"message": "no path drawn"})
    ctl.queue(CMD_PATH_RUN)
    if last:
        ctl.set_goal(last, "path end")
    ctl.status = f"running path, {n} points"
    return jsonify({"message": f"running path, {n} points"})

@app.route("/api/path/clear", methods=["POST"])
def path_clear():
    ctl.clear_path()
    ctl.clear_goal()
    ctl.status = "path cleared"
    return jsonify({"message": "path cleared"})

@app.route("/api/calib/set", methods=["POST"])
def calib_set():
    i = int(request.json["index"])
    x = float(request.json["x"])
    y = float(request.json["y"])
    ok, msg = ctl.set_calib(i, (x, y))
    ctl.status = msg
    return jsonify({"message": msg, "ok": ok})

@app.route("/api/calib/quick", methods=["POST"])
def calib_quick():
    ok, msg = ctl.quick_calibrate()
    ctl.status = msg
    return jsonify({"message": msg, "ok": ok})

@app.route("/api/calib/clear_point", methods=["POST"])
def calib_clear_point():
    i = int(request.json["index"])
    ok, msg = ctl.clear_calib_point(i)
    ctl.status = msg
    return jsonify({"message": msg, "ok": ok})

@app.route("/api/calib/reset", methods=["POST"])
def calib_reset():
    ctl.reset_calib()
    return jsonify({"message": f"calibration cleared, stand the robot on {CALIB_POINTS[0]}"})

@app.route("/api/zero", methods=["POST"])
def zero():
    ok, msg = ctl.zero_heading(request.get_json(silent=True))
    ctl.status = msg
    return jsonify({"message": msg, "ok": ok})

def main():
    global ctl
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=1)
    ap.add_argument("--port", default=None, help="paired MiniBot-Control outgoing Bluetooth COM port")
    ap.add_argument("--tag-id", type=int, default=None)
    ap.add_argument("--family", default="tag36h11")

    ap.add_argument("--tag-mm", type=float, default=127.0,
                    help="printed tag size in mm across the black border; 5 in = 127")
    ap.add_argument("--exposure", type=int, default=-5,
                    help="manual exposure, less negative is brighter; -1 is brightest")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--http-port", type=int, default=8000)
    args = ap.parse_args()

    ctl = Controller(args.camera, args.port, args.tag_id, args.family, args.width, args.height, args.exposure, args.tag_mm)
    print(f"\n  open http://localhost:{args.http_port}\n")
    app.run(host="127.0.0.1", port=args.http_port, threaded=True)

if __name__ == "__main__":
    main()
