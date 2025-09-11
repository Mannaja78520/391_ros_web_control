#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio, time, threading, cv2, logging, subprocess, re, glob, os, shlex, json
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame

# ================== CONFIG ==================
HOST, PORT = "0.0.0.0", 8000
SCALE = 1.0
PACE_FPS = 0
JPEG_QUALITY = 85
IDLE_CLOSE_SEC = 300
# ===========================================

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("webrtc")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TPL_DIR  = os.path.join(BASE_DIR, "templates")
STA_DIR  = os.path.join(BASE_DIR, "static")

# ---------- utils ----------
def run_cmd(cmd):
    try:
        out = subprocess.check_output(shlex.split(cmd), text=True, stderr=subprocess.STDOUT)
        return True, out
    except subprocess.CalledProcessError as e:
        return False, e.output

def list_v4l2_devices():
    items = []
    ok, out = run_cmd("v4l2-ctl --list-devices")
    if ok and out:
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
    if not items:
        for p in sorted(glob.glob("/dev/video*")):
            m = re.search(r"/dev/video(\d+)", p)
            if not m: continue
            idx = int(m.group(1))
            items.append({"index": idx, "path": p, "name": "VIDEO DEVICE"})
    # uniq
    seen=set(); uniq=[]
    for it in sorted(items, key=lambda x: x["index"]):
        if it["index"] in seen: continue
        seen.add(it["index"]); uniq.append(it)
    return uniq

def fourcc_from_cap(cap) -> str:
    try:
        fcc_i = int(cap.get(cv2.CAP_PROP_FOURCC))
        return "".join([chr((fcc_i >> 8*i) & 0xFF) for i in range(4)]).strip()
    except Exception:
        return ""

def parse_list_formats_ext(text: str):
    """
    แปลงผลลัพธ์จาก `v4l2-ctl --list-formats-ext` เป็นโครงสร้าง:
    {
      "MJPG": {
        "description": "Motion-JPEG, compressed",
        "sizes": [
          {"w":1280,"h":720,"fps":[30,25,20]},
          ...
        ]
      },
      "YUYV": { ... }
    }
    """
    formats = {}
    cur_fourcc = None
    cur_desc = None
    cur_size = None

    # ตัวอย่างบรรทัดที่ต้อง match:
    # [0]: 'MJPG' (Motion-JPEG, compressed)
    #     Size: Discrete 1280x720
    #         Interval: Discrete 0.033s (30.000 fps)
    #         Interval: Discrete 0.040s (25.000 fps)
    re_fmt = re.compile(r"^\s*\[\d+\]:\s+'([A-Z0-9]{4})'\s+\((.+)\)")
    re_size = re.compile(r"^\s*Size:\s*Discrete\s+(\d+)x(\d+)")
    re_fps = re.compile(r"^\s*Interval:\s*Discrete\s+[0-9\.]+s\s+\(([0-9\.]+)\s+fps\)")

    for line in text.splitlines():
        m = re_fmt.match(line)
        if m:
            cur_fourcc = m.group(1).upper()
            cur_desc = m.group(2).strip()
            if cur_fourcc not in formats:
                formats[cur_fourcc] = {"description": cur_desc, "sizes": []}
            continue

        if cur_fourcc:
            m2 = re_size.match(line)
            if m2:
                w = int(m2.group(1)); h = int(m2.group(2))
                cur_size = {"w": w, "h": h, "fps": []}
                formats[cur_fourcc]["sizes"].append(cur_size)
                continue

            m3 = re_fps.match(line)
            if m3 and cur_size is not None:
                try:
                    fps_val = float(m3.group(1))
                    # ปัดเป็น int ถ้าใกล้จำนวนเต็ม
                    if abs(round(fps_val) - fps_val) < 1e-3:
                        fps_val = int(round(fps_val))
                    if fps_val not in cur_size["fps"]:
                        cur_size["fps"].append(fps_val)
                except:
                    pass

    # จัดเรียงให้สวย (ใหญ่→เล็ก, fps มาก→น้อย)
    for fcc in formats:
        formats[fcc]["sizes"].sort(key=lambda s: (s["w"]*s["h"], s["w"]), reverse=True)
        for sz in formats[fcc]["sizes"]:
            sz["fps"] = sorted(sz["fps"], reverse=True)
    return formats

# ---------- camera ----------
class ZeroBufferCamera:
    def __init__(self, src: int):
        self.src = src
        self.cap_lock = threading.Lock()
        self.frame = None
        self.lock = threading.Lock()
        self.stopped = False
        self.last_used = time.time()
        self.cap = None
        self._open()
        threading.Thread(target=self._reader, daemon=True).start()
        logger.info(f"[cam {src}] opened")

    def _open(self):
        cap = cv2.VideoCapture(self.src)
        if not cap.isOpened():
            raise RuntimeError(f"เปิดกล้องไม่สำเร็จ: /dev/video{self.src}")
        with self.cap_lock:
            if self.cap is not None:
                try: self.cap.release()
                except: pass
            self.cap = cap

    def apply_once(self, w=None, h=None, fps=None, fourcc=None):
        with self.cap_lock:
            try:
                if self.cap: self.cap.release()
            except:
                pass
        fmt_args = []
        if w and h and fourcc:
            fmt_args.append(f"--set-fmt-video=width={int(w)},height={int(h)},pixelformat={fourcc.upper()}")
        elif w and h:
            fmt_args.append(f"--set-fmt-video=width={int(w)},height={int(h)}")
        elif fourcc:
            fmt_args.append(f"--set-fmt-video=pixelformat={fourcc.upper()}")
        if fps:
            fmt_args.append(f"--set-parm={float(fps)}")
        for arg in fmt_args:
            cmd = f"v4l2-ctl -d /dev/video{self.src} {arg}"
            ok, out = run_cmd(cmd)
            logger.info(f"[cam {self.src}] {cmd} -> ok={ok}\n{(out or '').strip()}")

        cap = cv2.VideoCapture(self.src)
        if not cap.isOpened():
            raise RuntimeError(f"เปิดกล้องไม่สำเร็จหลัง set: /dev/video{self.src}")
        with self.cap_lock:
            self.cap = cap

        with self.cap_lock:
            rw = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            rh = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            rf = float(self.cap.get(cv2.CAP_PROP_FPS))
            fcc = fourcc_from_cap(self.cap)
        logger.info(f"[cam {self.src}] applied(actual): {rw}x{rh}@{rf:.1f} fourcc={fcc or 'n/a'}")

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

    def info(self):
        with self.cap_lock:
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            f = float(self.cap.get(cv2.CAP_PROP_FPS))
            fcc = fourcc_from_cap(self.cap)
        return {"w": w, "h": h, "fps": f, "fourcc": fcc}

    def close(self):
        self.stopped = True
        time.sleep(0.05)
        with self.cap_lock:
            try: self.cap.release()
            except: pass
        logger.info(f"[cam {self.src}] closed")

class CameraPool:
    def __init__(self):
        self.cams = {}
        self.lock = threading.Lock()

    def get(self, index: int) -> ZeroBufferCamera:
        with self.lock:
            cam = self.cams.get(index)
            if cam is None:
                cam = ZeroBufferCamera(index)
                self.cams[index] = cam
            cam.last_used = time.time()
            return cam

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

# ---------- webrtc track ----------
class LatestFrameTrack(VideoStreamTrack):
    kind = "video"
    def __init__(self, cam: ZeroBufferCamera):
        super().__init__()
        self.cam = cam
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

        pts, tb = await self.next_timestamp()
        vf.pts, vf.time_base = pts, tb
        self.cam.last_used = time.time()
        return vf

# ---------- handlers ----------
async def index(request):
    return web.FileResponse(path=os.path.join(TPL_DIR, "index.html"))

def setup_static(app: web.Application):
    app.router.add_static("/static", path=STA_DIR, name="static")

async def devices(request):
    CAM_POOL.cleanup_idle()
    return web.json_response({"devices": list_v4l2_devices()})

async def snapshot(request):
    idx_str = request.query.get("index", "")
    if not idx_str.isdigit():
        return web.Response(text="query ?index=<int> required", status=400)
    idx = int(idx_str)
    f = CAM_POOL.get(idx).latest()
    if f is None:
        return web.Response(text="no frame yet", status=503)
    if SCALE != 1.0:
        f = cv2.resize(f, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        return web.Response(text="encode fail", status=500)
    return web.Response(body=buf.tobytes(), content_type="image/jpeg")

async def cam_info(request):
    s = request.query.get("index","")
    if not s.isdigit():
        return web.json_response({"ok": False, "error": "need ?index"}, status=400)
    idx = int(s)
    cam = CAM_POOL.get(idx)
    info = cam.info()
    return web.json_response({"ok": True, **info})

async def cam_formats(request):
    s = request.query.get("index","")
    if not s.isdigit():
        return web.json_response({"ok": False, "error": "need ?index"}, status=400)
    idx = int(s)
    ok, out = run_cmd(f"v4l2-ctl -d /dev/video{idx} --list-formats-ext")
    if not ok:
        return web.json_response({"ok": False, "error": out or "v4l2-ctl error"}, status=500)
    try:
        formats = parse_list_formats_ext(out)
        return web.json_response({"ok": True, "formats": formats})
    except Exception as e:
        return web.json_response({"ok": False, "error": str(e)}, status=500)

async def offer(request):
    q = request.query
    idx_str = q.get("index", "")
    if not idx_str.isdigit():
        return web.Response(text="query ?index=<int> required", status=400)
    idx = int(idx_str)

    w   = q.get("w") or q.get("width")
    h   = q.get("h") or q.get("height")
    fps = q.get("fps")
    cc  = q.get("fourcc")

    cam = CAM_POOL.get(idx)
    cam.apply_once(w=w, h=h, fps=fps, fourcc=cc)

    sdp = await request.text()
    pc = RTCPeerConnection()
    request.app["pcs"].add(pc)
    logger.info("PC create. Total: %d (cam %d)", len(request.app["pcs"]), idx)

    @pc.on("iceconnectionstatechange")
    async def on_ice():
        logger.info("ICE: %s", pc.iceConnectionState)
        if pc.iceConnectionState in ("failed","disconnected","closed"):
            await pc.close(); request.app["pcs"].discard(pc)

    @pc.on("connectionstatechange")
    async def on_conn():
        logger.info("Conn: %s", pc.connectionState)
        if pc.connectionState in ("failed","closed"):
            await pc.close(); request.app["pcs"].discard(pc)

    await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
    pc.addTrack(LatestFrameTrack(cam))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return web.Response(text=pc.localDescription.sdp, content_type="application/sdp")

# ---------- lifecycle ----------
async def on_startup(app):
    app["pcs"] = set()

async def on_shutdown(app):
    for pc in list(app["pcs"]):
        await pc.close()
    app["pcs"].clear()
    for cam in list(CAM_POOL.cams.values()):
        cam.close()
    CAM_POOL.cams.clear()

def main():
    app = web.Application()
    app.add_routes([
        web.get("/", index),
        web.get("/devices", devices),
        web.get("/snapshot", snapshot),
        web.get("/cam_info", cam_info),
        web.get("/cam_formats", cam_formats),  # ★ ใช้เติม dropdown ตามจริง
        web.post("/offer", offer),
    ])
    setup_static(app)
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    web.run_app(app, host=HOST, port=PORT)

if __name__ == "__main__":
    main()
