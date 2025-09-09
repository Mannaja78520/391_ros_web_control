#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2, threading, time
from flask import Flask, Response, render_template_string

CAMERA_INDEX = 0          # เปลี่ยนเป็น RTSP/URL ได้ เช่น "rtsp://user:pass@ip/..."
FRAME_WIDTH  = 1280       # ปรับได้
FRAME_HEIGHT = 720
TARGET_FPS   = 30

app = Flask(__name__)

class Camera:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError("เปิดกล้องไม่สำเร็จ")

        # ลดการหน่วงบัฟเฟอร์จากไดรเวอร์ (ถ้าซัพพอร์ต)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

        self.lock   = threading.Lock()
        self.frame  = None
        self.stopped = False

        t = threading.Thread(target=self._reader, daemon=True)
        t.start()

    def _reader(self):
        """อ่านกล้องตลอดเวลา เก็บไว้แค่เฟรมล่าสุด -> ไม่มีการกองบัฟเฟอร์"""
        while not self.stopped:
            # ใช้ grab() เพื่อดึงเฟรมล่าสุดเร็ว ๆ แล้ว read() เอาเฉพาะเฟรมล่าสุด
            self.cap.grab()
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            with self.lock:
                self.frame = frame

    def get_jpeg(self):
        """คืนค่า JPEG ของเฟรมล่าสุด (อาจเป็น None ถ้าเพิ่งเริ่ม)"""
        with self.lock:
            f = None if self.frame is None else self.frame.copy()
        if f is None:
            return None
        ok, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            return None
        return buf.tobytes()

    def stop(self):
        self.stopped = True
        time.sleep(0.05)
        self.cap.release()

cam = Camera(CAMERA_INDEX)

PAGE = """
<!doctype html>
<html lang="th">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Realtime Camera (No-Buffer)</title>
  <style>
    body{font-family:system-ui,Segoe UI,Arial,sans-serif;margin:0;padding:16px;background:#111;color:#eee}
    .wrap{max-width:1000px;margin:0 auto}
    img{max-width:100%;height:auto;border-radius:12px;display:block}
    .row{display:flex;gap:12px;align-items:center;flex-wrap:wrap}
    .chip{background:#222;border:1px solid #333;border-radius:999px;padding:6px 12px}
  </style>
</head>
<body>
  <div class="wrap">
    <h2>Realtime Camera — Zero Buffer</h2>
    <div class="row">
      <div class="chip">MJPEG stream</div>
      <div class="chip">resolution: {{w}}×{{h}}</div>
      <div class="chip">fps target: {{fps}}</div>
    </div>
    <p>ถ้าไม่ขึ้นภาพ ลองรีเฟรช หรือเช็คสิทธิ์กล้อง/พอร์ต</p>
    <img src="/video" alt="camera stream">
  </div>
</body>
</html>
"""

@app.after_request
def no_cache(resp):
    # ปิด cache ทุกอย่าง ลดการค้างเฟรม
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp

@app.route("/")
def index():
    return render_template_string(PAGE, w=FRAME_WIDTH, h=FRAME_HEIGHT, fps=TARGET_FPS)

def mjpeg_generator():
    boundary = b'--frame'
    while True:
        jpg = cam.get_jpeg()
        if jpg is None:
            time.sleep(0.01)
            continue
        yield (boundary + b"\r\n"
               b"Content-Type: image/jpeg\r\n"
               b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" +
               jpg + b"\r\n")

@app.route("/video")
def video():
    return Response(mjpeg_generator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    try:
        # dev: localhost เท่านั้นก่อน; ถ้าจะให้เครื่องอื่นดู ใช้ host="0.0.0.0" (ดูข้อ 2)
        app.run(host="127.0.0.1", port=5000, threaded=True)
    finally:
        cam.stop()
