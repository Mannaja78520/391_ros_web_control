#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Realtime zero-buffer camera → WebRTC (aiortc) + snapshot (HTTP)
- ดึงเฉพาะ "เฟรมล่าสุด" ลดอาการหน่วง/กองคิว
- ใช้ V4L2 + MJPG เพื่อลดภาระ CPU (เว็บแคมส่วนใหญ่ลื่นขึ้น)
- มี SCALE ย่อภาพก่อนส่ง (ทั้ง snapshot และ WebRTC)
- pace เฟรมให้คงที่ตาม target FPS
"""

import asyncio, time, threading, cv2, av, logging
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame

# =========================
# CONFIG
# =========================
WIDTH  = 1920     # ความละเอียด "ต้นทาง" จากกล้อง
HEIGHT = 1080
FPS    = 60      # target fps ทั้งฝั่ง capture และส่ง
SCALE  = 1.0    # 1.0=เต็ม, 0.5=ครึ่ง, 0.25=หนึ่งในสี่

HOST   = "0.0.0.0"
PORT   = 8000
CAMERA_INDEX = 0  # เปลี่ยนเป็น 1/2 หรือ RTSP URL ได้

# =========================
# LOGGING
# =========================
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("webrtc")

# =========================
# CAMERA (Zero-buffer)
# =========================
class ZeroBufferCamera:
    def __init__(self, src=0, width=1280, height=720, fps=30):
        # ใช้ V4L2 โดยตรง (Linux) เพื่อเสถียรภาพที่ดีขึ้น
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("เปิดกล้องไม่สำเร็จ")

        # พยายามใช้ MJPG จากกล้องเพื่อลดภาระ decode/encode
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # ตั้งค่าพื้นฐาน + ลดบัฟเฟอร์
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 5)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        # ลดกระตุกจากการวิ่งค่า exposure/focus (ถ้ากล้องรองรับ)
        # หมายเหตุ: บางรุ่นต้องใช้ 0.25 แทน 1 (ขึ้นกับไดรเวอร์)
        try:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        except Exception:
            pass
        try:
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        except Exception:
            pass

        self.frame = None
        self.lock = threading.Lock()
        self.stopped = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        # อ่านให้เร็วที่สุด: grab → read แล้วเก็บแค่เฟรมล่าสุด
        while not self.stopped:
            self.cap.grab()
            ok, f = self.cap.read()
            if ok:
                with self.lock:
                    self.frame = f
            else:
                time.sleep(0.005)

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def close(self):
        self.stopped = True
        time.sleep(0.05)
        try:
            self.cap.release()
        except Exception:
            pass

# =========================
# WebRTC Track (pace + scale)
# =========================
class LatestFrameTrack(VideoStreamTrack):
    kind = "video"
    def __init__(self, cam: ZeroBufferCamera, fps: int = FPS):
        super().__init__()
        self.cam = cam
        self._frame_interval = 1.0 / max(1, fps)
        self._next_t = time.monotonic()

    async def recv(self):
        # pace ตาม target fps
        now = time.monotonic()
        delay = self._next_t - now
        if delay > 0:
            await asyncio.sleep(delay)
        self._next_t += self._frame_interval

        frame = self.cam.latest()
        if frame is None:
            # เฟรมกันล้ม (สีดำ) + รองรับ SCALE
            base_w, base_h = 640, 360
            w = int(base_w * SCALE)
            h = int(base_h * SCALE)
            w = max(2, w); h = max(2, h)
            vf = VideoFrame(width=w, height=h, format="bgr24")
        else:
            # ย่อภาพก่อนสร้าง VideoFrame
            if SCALE != 1.0:
                frame = cv2.resize(frame, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_AREA)
            vf = VideoFrame.from_ndarray(frame, format="bgr24")

        # ใส่ timestamp ให้ถูกต้องสำหรับ aiortc
        pts, time_base = await self.next_timestamp()
        vf.pts = pts
        vf.time_base = time_base
        return vf

# =========================
# HTTP UI
# =========================
INDEX = f"""<!doctype html><meta charset=utf-8>
<title>WebRTC Zero-Buffer</title>
<style>
  body{{font-family:system-ui;background:#111;color:#eee;margin:0;padding:16px}}
  video{{width:100%;background:#000;border-radius:12px}}
  .chip{{display:inline-block;padding:6px 12px;border:1px solid #333;border-radius:999px;background:#222;margin-right:6px}}
</style>
<h2>WebRTC Zero-Buffer</h2>
<div class="chip">src: {CAMERA_INDEX}</div>
<div class="chip">cap: {WIDTH}×{HEIGHT}@{FPS}</div>
<div class="chip">scale: {SCALE}</div>
<div id=status class="chip">idle</div>
<p><button id=s>Start</button> <button id=x>Stop</button></p>
<video id=v autoplay playsinline muted></video>
<script>
let pc=null, v=document.getElementById('v'), st=document.getElementById('status');
function setS(s){{st.textContent=s}}
async function start(){{
  if(pc) return;
  setS('connecting...');
  pc=new RTCPeerConnection();
  pc.addTransceiver('video',{{direction:'recvonly'}});
  pc.ontrack=(e)=>{{v.srcObject=e.streams[0]; setS('playing')}};
  pc.oniceconnectionstatechange=()=>setS(pc.iceConnectionState);
  const offer=await pc.createOffer();
  await pc.setLocalDescription(offer);
  const r=await fetch('/offer',{{method:'POST',headers:{{'Content-Type':'application/sdp'}},body:offer.sdp}});
  const ans=await r.text();
  await pc.setRemoteDescription({{type:'answer',sdp:ans}});
}}
function stop(){{
  if(!pc) return;
  pc.getTransceivers().forEach(t=>t.stop&&t.stop());
  pc.close(); pc=null; v.srcObject=null; setS('stopped');
}}
document.getElementById('s').onclick=start;
document.getElementById('x').onclick=stop;
</script>
"""

pcs = set()

async def index(request):   return web.Response(text=INDEX, content_type="text/html")
async def health(request):  return web.json_response({"ok": True})

async def snapshot(request):
    f = request.app["camera"].latest()
    if f is None:
        return web.Response(text="no frame yet", status=503)
    if SCALE != 1.0:
        f = cv2.resize(f, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        return web.Response(text="encode fail", status=500)
    return web.Response(body=buf.tobytes(), content_type="image/jpeg")

async def offer(request):
    sdp = await request.text()
    pc = RTCPeerConnection()
    pcs.add(pc)
    logger.info("PC create. Total: %d", len(pcs))

    @pc.on("iceconnectionstatechange")
    async def on_ice():
        logger.info("ICE state: %s", pc.iceConnectionState)
        if pc.iceConnectionState in ("failed","disconnected","closed"):
            await pc.close(); pcs.discard(pc)

    @pc.on("connectionstatechange")
    async def on_conn():
        logger.info("Conn state: %s", pc.connectionState)
        if pc.connectionState in ("failed","closed"):
            await pc.close(); pcs.discard(pc)

    await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))

    # สร้าง track (ส่งตาม FPS config) แล้วให้เบราว์เซอร์ต่อ
    track = LatestFrameTrack(request.app["camera"], fps=FPS)
    pc.addTrack(track)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return web.Response(text=pc.localDescription.sdp, content_type="application/sdp")

async def on_shutdown(app):
    for pc in list(pcs):
        await pc.close()
    pcs.clear()
    app["camera"].close()

def main():
    cam = ZeroBufferCamera(src=CAMERA_INDEX, width=WIDTH, height=HEIGHT, fps=FPS)
    app = web.Application()
    app["camera"] = cam
    app.router.add_get("/", index)
    app.router.add_get("/health", health)
    app.router.add_get("/snapshot", snapshot)
    app.router.add_post("/offer", offer)
    app.on_shutdown.append(on_shutdown)
    web.run_app(app, host=HOST, port=PORT)

if __name__ == "__main__":
    main()
