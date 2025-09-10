#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio, time, threading, cv2, logging, subprocess, re, glob, os, json
from aiohttp import web, WSMsgType
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame

from ros_thread import start_ros   # ✅ ใช้ start_ros(push_fn) เวอร์ชันใหม่

# ============== CONFIG ==============
HOST, PORT = "0.0.0.0", 8000
SCALE = 1.0            # ย่อเฉพาะตอน "ส่งออก" (ไม่กระทบกล้อง)
PACE_FPS = 0           # 0=ไม่ pace, >0 จะ pace ตามค่านี้ (เช่น 24/30)
JPEG_QUALITY = 85      # คุณภาพ /snapshot
IDLE_CLOSE_SEC = 300   # ปิดกล้องถ้าไม่ถูกใช้เกิน N วินาที
# ====================================

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("webrtc")

# ========== Utilities: list V4L2 devices ==========
def list_v4l2_devices():
    items = []
    try:
        out = subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True, stderr=subprocess.STDOUT)
        blocks = [b for b in out.strip().split("\n\n") if b.strip()]
        for b in blocks:
            lines = [l for l in b.splitlines() if l.strip()]
            name = lines[0].strip()
            paths = [l.strip() for l in lines[1:] if "/dev/video" in l]
            for p in paths:
                m = re.search(r"/dev/video(\d+)", p)
                if not m: continue
                idx = int(m.group(1))
                items.append({"index": idx, "path": p, "name": name})
    except Exception:
        pass

    if not items:
        for p in sorted(glob.glob("/dev/video*")):
            m = re.search(r"/dev/video(\d+)", p)
            if not m: continue
            idx = int(m.group(1))
            try:
                name = os.path.basename(os.path.realpath(f"/sys/class/video4linux/video{idx}/device/..")).upper()
            except Exception:
                name = "VIDEO DEVICE"
            items.append({"index": idx, "path": p, "name": name})

    seen=set(); uniq=[]
    for it in sorted(items, key=lambda x: x["index"]):
        if it["index"] in seen: continue
        seen.add(it["index"]); uniq.append(it)
    return uniq

# ========== Camera single ==========
class ZeroBufferCamera:
    def __init__(self, src: int):
        self.src = src
        self.cap_lock = threading.Lock()
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            raise RuntimeError(f"เปิดกล้องไม่สำเร็จ: /dev/video{src}")
        self.cap = cap
        self.frame = None
        self.lock = threading.Lock()
        self.stopped = False
        self.last_used = time.time()
        threading.Thread(target=self._reader, daemon=True).start()
        logger.info(f"[cam {src}] opened")

    def _reader(self):
        while not self.stopped:
            with self.cap_lock:
                okg = self.cap.grab()
                ok, f = self.cap.read() if okg else (False, None)
            if ok:
                with self.lock:
                    self.frame = f
                self.last_used = time.time()
            else:
                time.sleep(0.005)

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def close(self):
        self.stopped = True
        time.sleep(0.05)
        with self.cap_lock:
            try: self.cap.release()
            except Exception: pass
        logger.info(f"[cam {self.src}] closed")

# ========== Camera pool ==========
class CameraPool:
    def __init__(self):
        self.cams = {}       # index -> ZeroBufferCamera
        self.lock = threading.Lock()

    def get(self, index: int) -> ZeroBufferCamera:
        with self.lock:
            cam = self.cams.get(index)
            if cam is None:
                cam = ZeroBufferCamera(index)
                self.cams[index] = cam
            cam.last_used = time.time()
            return cam

    def snapshot_frame(self, index: int):
        cam = self.get(index)
        return cam.latest()

    def cleanup_idle(self, idle_sec: int = IDLE_CLOSE_SEC):
        now = time.time()
        to_close = []
        with self.lock:
            for idx, cam in self.cams.items():
                if now - cam.last_used > idle_sec:
                    to_close.append(idx)
            for idx in to_close:
                self.cams[idx].close()
                del self.cams[idx]

CAM_POOL = CameraPool()

# ========== WebRTC track (one camera per track) ==========
class LatestFrameTrack(VideoStreamTrack):
    kind = "video"
    def __init__(self, index: int):
        super().__init__()
        self.index = index
        self.cam = CAM_POOL.get(index)
        self._interval = (1.0 / PACE_FPS) if PACE_FPS and PACE_FPS > 0 else 0.0
        self._next_t = time.monotonic()

    async def recv(self):
        if self._interval > 0:
            now = time.monotonic()
            delay = self._next_t - now
            if delay > 0:
                await asyncio.sleep(delay)
            self._next_t += self._interval

        frame = self.cam.latest()
        if frame is None:
            w, h = 640, 360
            if SCALE != 1.0:
                w = max(2, int(w * SCALE)); h = max(2, int(h * SCALE))
            vf = VideoFrame(width=w, height=h, format="bgr24")
        else:
            if SCALE != 1.0:
                frame = cv2.resize(frame, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_AREA)
            vf = VideoFrame.from_ndarray(frame, format="bgr24")

        pts, time_base = await self.next_timestamp()
        vf.pts, vf.time_base = pts, time_base
        self.cam.last_used = time.time()
        return vf

# ========== Web UI ==========
async def index(request):
    return web.FileResponse(path="templates/index.html")

def setup_static(app: web.Application):
    app.router.add_static("/static", path="static", name="static")

# Devices API
async def devices(request):
    CAM_POOL.cleanup_idle()
    return web.json_response({"devices": list_v4l2_devices()})

# Snapshot per camera
async def snapshot(request):
    idx_str = request.query.get("index", "")
    if not idx_str.isdigit():
        return web.Response(text="query ?index=<int> required", status=400)
    idx = int(idx_str)
    f = CAM_POOL.snapshot_frame(idx)
    if f is None:
        return web.Response(text="no frame yet", status=503)
    if SCALE != 1.0:
        f = cv2.resize(f, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        return web.Response(text="encode fail", status=500)
    return web.Response(body=buf.tobytes(), content_type="image/jpeg")

# WebRTC offer per camera
async def offer(request):
    idx_str = request.query.get("index", "")
    if not idx_str.isdigit():
        return web.Response(text="query ?index=<int> required", status=400)
    idx = int(idx_str)

    sdp = await request.text()
    pc = RTCPeerConnection()
    request.app["pcs"].add(pc)
    logger.info("PC create. Total: %d (cam %d)", len(request.app["pcs"]), idx)

    @pc.on("iceconnectionstatechange")
    async def on_ice():
        logger.info("ICE state: %s", pc.iceConnectionState)
        if pc.iceConnectionState in ("failed","disconnected","closed"):
            await pc.close(); request.app["pcs"].discard(pc)

    @pc.on("connectionstatechange")
    async def on_conn():
        logger.info("Conn state: %s", pc.connectionState)
        if pc.connectionState in ("failed","closed"):
            await pc.close(); request.app["pcs"].discard(pc)

    track = LatestFrameTrack(index=idx)

    await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
    pc.addTrack(track)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(text=pc.localDescription.sdp, content_type="application/sdp")

# ======== WebSocket (ROS data push) ========
async def ws_ros(request):
    ws = web.WebSocketResponse(heartbeat=30.0)
    await ws.prepare(request)
    request.app["ws_clients"].add(ws)
    logger.info("WS connected. Total: %d", len(request.app["ws_clients"]))
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT and msg.data == "ping":
                await ws.send_str("pong")
    finally:
        request.app["ws_clients"].discard(ws)
        logger.info("WS closed. Total: %d", len(request.app["ws_clients"]))
    return ws

# ======== Broadcaster: consume queue -> broadcast to all WS ========
async def ros_broadcast_loop(app: web.Application):
    queue: asyncio.Queue = app["ros_queue"]
    ws_clients: set = app["ws_clients"]
    while True:
        data = await queue.get()  # dict from ROS thread
        text = json.dumps({"type":"ros","payload":data}, ensure_ascii=False)
        dead = []
        for ws in ws_clients:
            try:
                await ws.send_str(text)
            except Exception:
                dead.append(ws)
        for d in dead:
            ws_clients.discard(d)

# ======== Startup / Shutdown ========
async def on_startup(app):
    app["pcs"] = set()
    app["ws_clients"] = set()
    app["ros_queue"] = asyncio.Queue()
    app["ros_broadcast_task"] = asyncio.create_task(ros_broadcast_loop(app))

    # push_fn: ros_thread จะเรียกจากเธรด ROS → เรา enqueue เข้า asyncio queue อย่างปลอดภัย
    loop = asyncio.get_running_loop()
    def push_fn(payload: dict):
        loop.call_soon_threadsafe(app["ros_queue"].put_nowait, payload)

    # สตาร์ท ROS2 (เธรด) และเก็บตัว node/stop ถ้าจำเป็น
    app["ros_node"] = start_ros(push_fn)
    logger.info("ROS thread started")

async def on_shutdown(app):
    # ปิด WS
    for ws in list(app["ws_clients"]):
        await ws.close()
    app["ws_clients"].clear()

    # ปิด broadcaster task
    if "ros_broadcast_task" in app:
        app["ros_broadcast_task"].cancel()
        try: await app["ros_broadcast_task"]
        except: pass

    # ปิด PeerConnections
    if "pcs" in app:
        for pc in list(app["pcs"]):
            await pc.close()
        app["pcs"].clear()

    # ปิดกล้องทั้งหมด
    for cam in list(CAM_POOL.cams.values()):
        cam.close()
    CAM_POOL.cams.clear()

def main():
    app = web.Application()
    app.add_routes([
        web.get("/", index),
        web.get("/devices", devices),
        web.get("/snapshot", snapshot),      # /snapshot?index=N
        web.post("/offer", offer),           # /offer?index=N
        web.get("/ws", ws_ros),              # WebSocket สำหรับ ROS data
    ])
    setup_static(app)
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    web.run_app(app, host=HOST, port=PORT)

if __name__ == "__main__":
    main()
