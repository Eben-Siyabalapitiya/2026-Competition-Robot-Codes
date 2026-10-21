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
    python web_ui.py --port COM5
Then open http://localhost:8000
"""

import argparse
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request
from pupil_apriltags import Detector

from minibot_vision import (
    CALIB_POINTS,
    CMD_CANCEL,
    CMD_DISABLE,
    CMD_ENABLE,
    CMD_GOTO_TGT,
    CMD_GOTO_XY,
    CMD_NONE,
    FIELD_X_MM,
    FIELD_Y_MM,
    FieldMap,
    Link,
    tag_pose,
)

app = Flask(__name__)


class Controller:
    def __init__(self, camera, port, tag_id, family, width, height):
        self.cap = cv2.VideoCapture(camera, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.detector = Detector(families=family, nthreads=4, quad_decimate=1.0)
        self.fmap = FieldMap()
        self.link = Link(port)
        self.tag_id = tag_id

        self.lock = threading.Lock()
        self.frame = None          # latest annotated frame
        self.pose = None           # (x_mm, y_mm, heading_deg)
        self.tag_center = None     # image pixels
        self.enabled = False
        self.status = "not calibrated"
        self.calib_shots = []

        self.pending = None        # (cmd, gx, gy, target) queued from the page
        self.running = True

        threading.Thread(target=self.worker, daemon=True).start()

    # ------------------------------------------------------------- the loop
    def worker(self):
        last_send = 0.0
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            dets = [
                d for d in self.detector.detect(gray)
                if self.tag_id is None or d.tag_id == self.tag_id
            ]

            pose = None
            centre = None
            if dets:
                d = dets[0]
                centre = (float(d.center[0]), float(d.center[1]))
                cv2.polylines(frame, [np.array(d.corners, dtype=np.int32)], True, (0, 255, 0), 2)
                cv2.circle(frame, (int(centre[0]), int(centre[1])), 5, (0, 0, 255), -1)
                if self.fmap.ready:
                    pose = tag_pose(d, self.fmap)

            if self.fmap.ready:
                self.draw_field(frame)

            with self.lock:
                self.pose = pose
                self.tag_center = centre
                self.frame = frame
                pending = self.pending
                self.pending = None

            cmd, gx, gy, target = pending if pending else (CMD_NONE, 0, 0, 0)

            # pose out at ~50 Hz; this doubles as the link heartbeat, the robot
            # disables itself if it stops arriving
            now = time.time()
            if cmd != CMD_NONE or (now - last_send) > 0.02:
                self.link.send(pose or (0, 0, 0), pose is not None, cmd, gx, gy, target)
                last_send = now

    def draw_field(self, frame):
        corners = [(0, 0), (FIELD_X_MM, 0), (FIELD_X_MM, FIELD_Y_MM), (0, FIELD_Y_MM)]
        pts = np.array([self.fmap.to_image(p) for p in corners], dtype=np.int32)
        cv2.polylines(frame, [pts], True, (0, 220, 255), 2)
        for cxy in [(610, 610), (1219, 610)]:
            cv2.circle(frame, self.fmap.to_image(cxy), 8, (255, 120, 0), 2)
        for t in CALIB_POINTS:
            cv2.circle(frame, self.fmap.to_image(t), 5, (0, 255, 0), 1)

    # ------------------------------------------------------------- commands
    def queue(self, cmd, gx=0, gy=0, target=0):
        with self.lock:
            self.pending = (cmd, gx, gy, target)

    def capture_calib(self):
        with self.lock:
            centre = self.tag_center
        if centre is None:
            return False, "no tag visible"
        if len(self.calib_shots) >= len(CALIB_POINTS):
            return False, "already have four points"

        self.calib_shots.append(centre)
        n = len(self.calib_shots)
        if n < len(CALIB_POINTS):
            return True, f"captured {n}/4, move to {CALIB_POINTS[n]}"

        if self.fmap.solve(self.calib_shots, CALIB_POINTS):
            self.status = "calibrated"
            return True, "calibrated"
        self.calib_shots = []
        return False, "homography failed, start again"

    def reset_calib(self):
        self.calib_shots = []
        self.fmap.H = None
        self.fmap.H_inv = None
        self.status = "not calibrated"

    def zero_heading(self):
        with self.lock:
            pose = self.pose
        if not pose:
            return False, "no tag visible"
        self.fmap.heading_offset += pose[2]
        return True, f"heading zeroed, offset {self.fmap.heading_offset:.1f}"

    def jpeg(self):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes() if ok else None


ctl: Controller = None


PAGE = """
<!doctype html>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MiniBot Control</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; background:#111; color:#eee; font:15px system-ui, sans-serif; }
  .wrap { max-width:1100px; margin:0 auto; padding:16px; }
  h1 { font-size:18px; margin:0 0 12px; letter-spacing:.02em; }
  #view { position:relative; display:inline-block; max-width:100%; }
  #cam { max-width:100%; display:block; border-radius:8px; cursor:crosshair; }
  .row { display:flex; flex-wrap:wrap; gap:8px; margin:12px 0; }
  button { background:#222; color:#eee; border:1px solid #444; border-radius:6px;
           padding:10px 14px; font-size:15px; cursor:pointer; }
  button:hover { background:#2d2d2d; }
  .go  { border-color:#2a7; }
  .bad { border-color:#a33; }
  .bar { padding:10px 14px; border-radius:6px; background:#1a1a1a; border:1px solid #333; }
  .on  { color:#4d4; font-weight:600; }
  .off { color:#f55; font-weight:600; }
  code { color:#8cf; }
</style>
<div class="wrap">
  <h1>MiniBot Control</h1>

  <div id="view"><img id="cam" src="/stream"></div>

  <div class="row">
    <button class="go"  onclick="post('/api/enable')">ENABLE</button>
    <button class="bad" onclick="post('/api/disable')">DISABLE</button>
    <button onclick="post('/api/cancel')">Cancel goal</button>
  </div>

  <div class="row">
    <button onclick="target(1)">Corner blue A</button>
    <button onclick="target(2)">Corner blue B</button>
    <button onclick="target(5)">Centre blue</button>
    <button onclick="target(6)">Centre orange</button>
    <button onclick="target(3)">Corner orange A</button>
    <button onclick="target(4)">Corner orange B</button>
  </div>

  <div class="row">
    <button onclick="post('/api/calib/capture')">Capture calibration point</button>
    <button onclick="post('/api/calib/reset')">Reset calibration</button>
    <button onclick="post('/api/zero')">Zero heading</button>
  </div>

  <div class="bar" id="state">...</div>
  <p>Click anywhere on the video to send the robot there. Calibration: stand the
  robot on each known point, press <code>Capture</code>, four times.</p>
</div>

<script>
async function post(url, body) {
  const r = await fetch(url, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body || {})
  });
  const j = await r.json();
  if (j.message) document.getElementById('state').textContent = j.message;
}
function target(id) { post('/api/target', {id: id}); }

document.getElementById('cam').addEventListener('click', (e) => {
  const img = e.target;
  const rect = img.getBoundingClientRect();
  // the page may be showing the frame scaled, so convert back to real pixels
  const x = (e.clientX - rect.left) * (img.naturalWidth  / rect.width);
  const y = (e.clientY - rect.top)  * (img.naturalHeight / rect.height);
  post('/api/click', {x: x, y: y});
});

setInterval(async () => {
  const s = await (await fetch('/api/state')).json();
  const en = s.enabled ? '<span class="on">ENABLED</span>'
                       : '<span class="off">disabled</span>';
  const pose = s.pose
    ? `x ${s.pose[0].toFixed(0)} &nbsp; y ${s.pose[1].toFixed(0)} &nbsp; hdg ${s.pose[2].toFixed(1)}&deg;`
    : '<span class="off">NO TAG</span>';
  document.getElementById('state').innerHTML =
    `${en} &nbsp;|&nbsp; ${pose} &nbsp;|&nbsp; ${s.status}`;
}, 300);
</script>
"""


@app.route("/")
def index():
    return PAGE


@app.route("/stream")
def stream():
    def gen():
        while True:
            buf = ctl.jpeg()
            if buf is not None:
                yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + buf + b"\r\n"
            time.sleep(0.033)

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=f")


@app.route("/api/state")
def state():
    with ctl.lock:
        pose = ctl.pose
    return jsonify({
        "pose": list(pose) if pose else None,
        "enabled": ctl.enabled,
        "status": ctl.status,
        "calibrated": ctl.fmap.ready,
        "calib_points": len(ctl.calib_shots),
    })


@app.route("/api/enable", methods=["POST"])
def enable():
    ctl.enabled = True
    ctl.status = "ENABLE sent"
    ctl.queue(CMD_ENABLE)
    return jsonify({"message": "enabled"})


@app.route("/api/disable", methods=["POST"])
def disable():
    ctl.enabled = False
    ctl.status = "DISABLE sent"
    ctl.queue(CMD_DISABLE)
    return jsonify({"message": "disabled"})


@app.route("/api/cancel", methods=["POST"])
def cancel():
    ctl.status = "goal cancelled"
    ctl.queue(CMD_CANCEL)
    return jsonify({"message": "cancelled"})


@app.route("/api/target", methods=["POST"])
def target():
    tid = int(request.json.get("id", 0))
    if not ctl.fmap.ready:
        return jsonify({"message": "calibrate first"})
    ctl.status = f"driving to target {tid}"
    ctl.queue(CMD_GOTO_TGT, target=tid)
    return jsonify({"message": f"target {tid}"})


@app.route("/api/click", methods=["POST"])
def click():
    if not ctl.fmap.ready:
        return jsonify({"message": "calibrate first"})
    x = float(request.json["x"])
    y = float(request.json["y"])
    fx, fy = ctl.fmap.to_field((x, y))
    if not (0 <= fx <= FIELD_X_MM and 0 <= fy <= FIELD_Y_MM):
        return jsonify({"message": "that point is off the field"})
    ctl.status = f"driving to {fx:.0f},{fy:.0f}"
    ctl.queue(CMD_GOTO_XY, gx=fx, gy=fy)
    return jsonify({"message": f"going to {fx:.0f},{fy:.0f}"})


@app.route("/api/calib/capture", methods=["POST"])
def calib_capture():
    ok, msg = ctl.capture_calib()
    ctl.status = msg
    return jsonify({"message": msg, "ok": ok})


@app.route("/api/calib/reset", methods=["POST"])
def calib_reset():
    ctl.reset_calib()
    return jsonify({"message": f"calibration cleared, stand the robot on {CALIB_POINTS[0]}"})


@app.route("/api/zero", methods=["POST"])
def zero():
    ok, msg = ctl.zero_heading()
    ctl.status = msg
    return jsonify({"message": msg, "ok": ok})


def main():
    global ctl
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--port", default=None, help="dongle COM port, e.g. COM5")
    ap.add_argument("--tag-id", type=int, default=None)
    ap.add_argument("--family", default="tag36h11")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--http-port", type=int, default=8000)
    args = ap.parse_args()

    ctl = Controller(args.camera, args.port, args.tag_id, args.family, args.width, args.height)
    print(f"\n  open http://localhost:{args.http_port}\n")
    app.run(host="0.0.0.0", port=args.http_port, threaded=True)


if __name__ == "__main__":
    main()
