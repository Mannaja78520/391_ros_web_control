#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-camera realtime → WebRTC (aiortc) + snapshot (HTTP)
- ไม่ตั้งค่ากล้อง (ใช้ค่าจากไดรเวอร์เดิม)
- เปิดได้หลายกล้อง "พร้อมกัน" (หนึ่ง PeerConnection ต่อหนึ่งกล้อง)
- หน้าเว็บ: เลือกหลายกล้องและแสดงเป็นกริด, ต่อช่องมี FPS/Res/ปุ่ม Fit/1:1/Close
- แก้บั๊ก KeyError จาก str.format() ด้วย sentinel tokens (__SCALE__, __PACE__)
"""

import asyncio, time, threading, cv2, logging, subprocess, re, glob, os
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame

# ============== CONFIG ==============
HOST, PORT = "0.0.0.0", 8000
SCALE = 1.0            # ย่อเฉพาะตอน "ส่งออก" (ไม่กระทบกล้อง)
PACE_FPS = 0           # 0=ไม่ pace, >0 จะ pace ตามค่านี้ (เช่น 24/30)
JPEG_QUALITY = 85      # คุณภาพ /snapshot
# ====================================

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("webrtc")

# --------- Utilities: list V4L2 devices ----------
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
                if not m:
                    continue
                idx = int(m.group(1))
                items.append({"index": idx, "path": p, "name": name})
    except Exception:
        pass

    if not items:
        for p in sorted(glob.glob("/dev/video*")):
            m = re.search(r"/dev/video(\d+)", p)
            if not m:
                continue
            idx = int(m.group(1))
            try:
                name = os.path.basename(os.path.realpath(f"/sys/class/video4linux/video{idx}/device/..")).upper()
            except Exception:
                name = "VIDEO DEVICE"
            items.append({"index": idx, "path": p, "name": name})

    seen=set(); uniq=[]
    for it in sorted(items, key=lambda x: x["index"]):
        if it["index"] in seen:
            continue
        seen.add(it["index"]); uniq.append(it)
    return uniq

# --------------- Camera ----------------
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
            try:
                self.cap.release()
            except Exception:
                pass
        logger.info(f"[cam {self.src}] closed")

# กล้องหลายตัว: เปิดตามต้องการแล้วเก็บไว้ใน dict
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

    def cleanup_idle(self, idle_sec: int = 300):
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

# --------------- WebRTC Track (หนึ่งกล้อง ต่อหนึ่ง track) ---------------
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

# ---------------- UI & HTTP ----------------
INDEX_HTML = """<!doctype html><meta charset=utf-8>
<title>Multi-Cam WebRTC</title>
<style>
  :root{color-scheme: dark}
  body{font-family:system-ui,Segoe UI,Helvetica,Arial,sans-serif;background:#111;color:#eee;margin:0;padding:16px}
  .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
  .chip{display:inline-block;padding:6px 12px;border:1px solid #333;border-radius:999px;background:#222;margin:0 6px 6px 0}
  button, select{padding:8px 14px;border-radius:8px;border:1px solid #444;background:#1e1e1e;color:#eee;cursor:pointer}
  button:hover{background:#2a2a2a}
  .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:12px;margin-top:12px}
  .card{background:#151515;border:1px solid #2a2a2a;border-radius:12px;padding:10px}
  .toolbar{display:flex;gap:6px;align-items:center;justify-content:space-between;margin-bottom:8px}
  .vid{display:block;background:#000;border-radius:8px;width:auto;height:auto;max-width:100%;object-fit:contain}
  .btns{display:flex;gap:6px;flex-wrap:wrap}
  small{opacity:.8}
</style>

<h2>Multi-Cam WebRTC</h2>
<div class="row">
  <div class="chip">scale: __SCALE__</div>
  <div class="chip">pace_fps: __PACE__</div>
  <div id="status" class="chip">idle</div>
</div>

<p class="row">
  <select id="camSel"></select>
  <button id="refresh">Refresh</button>
  <button id="add">Add Stream</button>
  <small>เลือกกล้องแล้วกด Add เพื่อเพิ่มช่องวิดีโอหลายกล้องได้</small>
</p>

<div id="grid" class="grid"></div>

<script>
let tileIdSeq = 1;

async function loadDevices(){
  const r = await fetch('/devices'); 
  const js = await r.json();
  const sel = document.getElementById('camSel');
  sel.innerHTML='';
  js.devices.forEach(d=>{
    const opt=document.createElement('option');
    opt.value=d.index;
    opt.textContent=`/dev/video${d.index} — ${d.name}`;
    sel.appendChild(opt);
  });
}
document.getElementById('refresh').onclick = loadDevices;

document.getElementById('add').onclick = async ()=>{
  const sel = document.getElementById('camSel');
  if (!sel.value) return;
  addTile(parseInt(sel.value,10));
};

function addTile(index){
  const id = 'tile'+(tileIdSeq++);
  const grid = document.getElementById('grid');

  const card = document.createElement('div');
  card.className='card';
  card.id = id;

  card.innerHTML = `
    <div class="toolbar">
      <div class="row">
        <div class="chip">/dev/video${index}</div>
        <div id="${id}-fps" class="chip">fps: --</div>
        <div id="${id}-res" class="chip">res: --</div>
        <div id="${id}-state" class="chip">idle</div>
      </div>
      <div class="btns">
        <button id="${id}-start">Start</button>
        <button id="${id}-stop" disabled>Stop</button>
        <button id="${id}-fit" disabled>Fit</button>
        <button id="${id}-one" disabled>1:1</button>
        <button id="${id}-close">Close</button>
      </div>
    </div>
    <video id="${id}-v" class="vid" autoplay playsinline muted></video>
  `;
  grid.appendChild(card);

  const v   = document.getElementById(`${id}-v`);
  const st  = document.getElementById(`${id}-state`);
  const fps = document.getElementById(`${id}-fps`);
  const res = document.getElementById(`${id}-res`);
  const bStart = document.getElementById(`${id}-start`);
  const bStop  = document.getElementById(`${id}-stop`);
  const bFit   = document.getElementById(`${id}-fit`);
  const bOne   = document.getElementById(`${id}-one`);
  const bClose = document.getElementById(`${id}-close`);

  let pc = null;
  let stopFPS = null;

  function setState(s){ st.textContent = s; }

  async function start(){
    if (pc) return;
    setState('connecting');
    pc = new RTCPeerConnection();
    pc.addTransceiver('video',{direction:'recvonly'});
    pc.ontrack = (e)=>{
      v.srcObject = e.streams[0];
      setState('playing');
      v.onloadedmetadata = ()=>{
        setOneToOne();
        res.textContent = 'res: ' + v.videoWidth + 'x' + v.videoHeight;
        startFPSCounter();
      };
    };
    pc.oniceconnectionstatechange = ()=> setState(pc.iceConnectionState);

    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    const r = await fetch('/offer?index='+encodeURIComponent(index), {
      method:'POST', headers:{'Content-Type':'application/sdp'}, body:offer.sdp
    });
    const ans = await r.text();
    await pc.setRemoteDescription({type:'answer', sdp:ans});

    bStart.disabled = true; bStop.disabled=false; bFit.disabled=false; bOne.disabled=false;
  }

  function stop(){
    if (!pc) return;
    pc.getTransceivers().forEach(t=>t.stop&&t.stop());
    pc.close(); pc=null;
    v.srcObject=null;
    setState('stopped');
    fps.textContent='fps: --';
    res.textContent='res: --';
    if (stopFPS) stopFPS();
    bStart.disabled=false; bStop.disabled=true; bFit.disabled=true; bOne.disabled=true;
  }

  function setOneToOne(){
    const w=v.videoWidth||640, h=v.videoHeight||360;
    v.style.width  = w + 'px';
    v.style.height = h + 'px';
  }
  function setFit(){
    v.style.width  = '100%';
    v.style.height = 'auto';
  }

  function startFPSCounter(){
    if (stopFPS) stopFPS();

    if ('requestVideoFrameCallback' in HTMLVideoElement.prototype) {
      let last=performance.now(), count=0, running=true;
      const onFrame = (_now, _meta)=>{
        if (!running) return;
        count++;
        const now=performance.now();
        if (now-last>=1000) {
          const f=(count*1000)/(now-last);
          fps.textContent = 'fps: ' + f.toFixed(1);
          count=0; last=now;
        }
        v.requestVideoFrameCallback(onFrame);
      };
      v.requestVideoFrameCallback(onFrame);
      stopFPS = ()=>{ running=false; };
      return;
    }

    let last=performance.now(), count=0, rafId=null;
    const tick=()=>{
      if (!v || v.readyState<2) { rafId=requestAnimationFrame(tick); return; }
      count++;
      const now=performance.now();
      if (now-last>=1000) {
        const f=(count*1000)/(now-last);
        fps.textContent = 'fps: ' + f.toFixed(1);
        count=0; last=now;
      }
      rafId=requestAnimationFrame(tick);
    };
    rafId=requestAnimationFrame(tick);
    stopFPS = ()=>{ if (rafId) cancelAnimationFrame(rafId); };
  }

  bStart.onclick = start;
  bStop.onclick  = stop;
  bFit.onclick   = setFit;
  bOne.onclick   = setOneToOne;
  bClose.onclick = ()=>{ stop(); card.remove(); };

  // auto start เมื่อเพิ่ม tile
  start();
}

loadDevices();
</script>
"""

def render_index():
    # แทนค่าเฉพาะ sentinel tokens เท่านั้น
    pace = "off" if not PACE_FPS else PACE_FPS
    html = INDEX_HTML.replace("__SCALE__", str(SCALE)).replace("__PACE__", str(pace))
    return html

async def index(request):   return web.Response(text=render_index(), content_type="text/html")
async def health(request):  return web.json_response({"ok": True})

async def devices(request):
    CAM_POOL.cleanup_idle()
    return web.json_response({"devices": list_v4l2_devices()})

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

async def on_shutdown(app):
    for pc in list(app["pcs"]):
        await pc.close()
    app["pcs"].clear()
    for cam in list(CAM_POOL.cams.values()):
        cam.close()
    CAM_POOL.cams.clear()

def main():
    app = web.Application()
    app["pcs"] = set()
    app.router.add_get("/", index)
    app.router.add_get("/health", health)
    app.router.add_get("/devices", devices)
    app.router.add_get("/snapshot", snapshot)       # /snapshot?index=N
    app.router.add_post("/offer", offer)            # /offer?index=N
    app.on_shutdown.append(on_shutdown)
    web.run_app(app, host=HOST, port=PORT)

if __name__ == "__main__":
    main()
