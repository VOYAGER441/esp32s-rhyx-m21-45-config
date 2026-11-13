"""
ESP32-CAM UDP receiver
  - Shows the feed in an OpenCV window (set SHOW_WINDOW = False for headless)
  - Serves the feed on a web page:  http://<this-pc-ip>:5000/
  - Supports several cameras (each camera has its own CAMERA_ID)

pip install flask opencv-python numpy
"""
import socket
import struct
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, render_template_string

UDP_IP = "0.0.0.0"
UDP_PORT = 2222
HTTP_PORT = 5000
SHOW_WINDOW = True

HDR = struct.Struct("<BHHH")   # camID, frameID, chunkIdx, chunkCount

latest = {}                    # cam_id -> {"jpg": bytes, "seq": int, "fps": float}
lock = threading.Lock()


# ---------------- UDP receiver thread ----------------
def udp_receiver():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(2)
    print(f"Listening for ESP32-CAM on UDP port {UDP_PORT} ...")

    assembling = {}            # cam_id -> {"fid", "count", "parts"}
    fps_state = {}             # cam_id -> [window_start, frame_count]

    while True:
        try:
            data, _ = sock.recvfrom(2048)
        except socket.timeout:
            continue
        if len(data) <= HDR.size:
            continue

        cam, fid, idx, count = HDR.unpack_from(data)
        payload = data[HDR.size:]

        st = assembling.get(cam)
        if st is None or st["fid"] != fid:
            st = {"fid": fid, "count": count, "parts": {}}
            assembling[cam] = st
        st["parts"][idx] = payload

        if len(st["parts"]) == st["count"]:
            jpg = b"".join(st["parts"][i] for i in range(st["count"]))
            assembling[cam] = {"fid": None, "count": 0, "parts": {}}

            now = time.time()
            fs = fps_state.setdefault(cam, [now, 0, 0.0])
            fs[1] += 1
            if now - fs[0] >= 1.0:
                fs[2] = fs[1] / (now - fs[0])
                fs[0], fs[1] = now, 0

            with lock:
                prev = latest.get(cam, {"seq": 0})
                latest[cam] = {"jpg": jpg, "seq": prev["seq"] + 1, "fps": fs[2]}


# ---------------- Flask web server ----------------
app = Flask(__name__)

PAGE = """
<!doctype html><title>ESP32-CAM</title>
<style>
  body{background:#111;color:#eee;font-family:sans-serif;margin:0;padding:16px}
  .grid{display:flex;flex-wrap:wrap;gap:16px}
  .cam img{max-width:100%;width:640px;border:2px solid #333;border-radius:6px}
</style>
<h2>ESP32-CAM live feed</h2>
<div class="grid">
{% for cid in cams %}
  <div class="cam"><div>Camera {{ cid }}</div><img src="/video/{{ cid }}"></div>
{% else %}
  <p>No camera streaming yet - refresh once the ESP32 is sending.</p>
{% endfor %}
</div>
"""


@app.route("/")
def index():
    with lock:
        cams = sorted(latest.keys())
    return render_template_string(PAGE, cams=cams)


def mjpeg_generator(cam_id):
    last_seq = -1
    while True:
        with lock:
            entry = latest.get(cam_id)
        if entry and entry["seq"] != last_seq:
            last_seq = entry["seq"]
            jpg = entry["jpg"]
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                   b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
        else:
            time.sleep(0.01)


@app.route("/video/<int:cam_id>")
def video(cam_id):
    return Response(mjpeg_generator(cam_id),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


def run_web():
    app.run(host="0.0.0.0", port=HTTP_PORT, threaded=True, use_reloader=False)


# ---------------- main ----------------
def main():
    threading.Thread(target=udp_receiver, daemon=True).start()
    threading.Thread(target=run_web, daemon=True).start()
    print(f"Web viewer: http://localhost:{HTTP_PORT}/")

    if not SHOW_WINDOW:
        while True:
            time.sleep(1)

    seen = {}
    opened = set()
    while True:
        with lock:
            snapshot = {k: dict(v) for k, v in latest.items()}

        for cam, entry in snapshot.items():
            if seen.get(cam) == entry["seq"]:
                continue
            seen[cam] = entry["seq"]

            img = cv2.imdecode(np.frombuffer(entry["jpg"], np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            cv2.putText(img, f"CAM {cam}  FPS: {entry['fps']:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            win = f"ESP32-CAM {cam}"
            if win not in opened:
                cv2.namedWindow(win, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(win, 960, 720)
                opened.add(win)
            cv2.imshow(win, img)

        if cv2.waitKey(5) == 27:   # ESC
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()