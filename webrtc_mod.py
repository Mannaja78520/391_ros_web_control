import asyncio, time, threading, os
import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame

WEBRTC_SCALE = float(os.environ.get("WEBRTC_SCALE", "1.0"))
WEBRTC_MAX_FPS = int(os.environ.get("WEBRTC_MAX_FPS", "20"))

_get_latest_frame = None
_loop = None
_thread = None
_pcs = set()


def init_webrtc(get_latest_frame):
    global _get_latest_frame, _loop, _thread
    _get_latest_frame = get_latest_frame
    if _loop is not None:
        return
    _loop = asyncio.new_event_loop()
    def run():
        asyncio.set_event_loop(_loop)
        _loop.run_forever()
    _thread = threading.Thread(target=run, daemon=True)
    _thread.start()


class SlotVideoTrack(VideoStreamTrack):
    kind = "video"
    def __init__(self, slot: int):
        super().__init__()
        self.slot = slot
        self._interval = (1.0 / WEBRTC_MAX_FPS) if WEBRTC_MAX_FPS and WEBRTC_MAX_FPS > 0 else 0.0
        self._next_t = time.monotonic()

    async def recv(self):
        if self._interval > 0:
            now = time.monotonic()
            delay = self._next_t - now
            if delay > 0:
                await asyncio.sleep(delay)
            self._next_t += self._interval

        frame = _get_latest_frame(self.slot) if _get_latest_frame else None
        if frame is None:
            h, w = 360, 640
            if WEBRTC_SCALE and WEBRTC_SCALE != 1.0:
                w = max(2, int(w * WEBRTC_SCALE)); h = max(2, int(h * WEBRTC_SCALE))
            blank = np.zeros((h, w, 3), dtype=np.uint8)
            vf = VideoFrame.from_ndarray(blank, format="bgr24")
        else:
            if WEBRTC_SCALE and WEBRTC_SCALE != 1.0:
                frame = cv2.resize(frame, None, fx=WEBRTC_SCALE, fy=WEBRTC_SCALE, interpolation=cv2.INTER_AREA)
            vf = VideoFrame.from_ndarray(frame, format="bgr24")
        pts, time_base = await self.next_timestamp()
        vf.pts, vf.time_base = pts, time_base
        return vf


async def _handle_offer_async(slot: int, sdp: str) -> str:
    pc = RTCPeerConnection()
    _pcs.add(pc)

    @pc.on("iceconnectionstatechange")
    async def on_ice():
        if pc.iceConnectionState in ("failed", "disconnected", "closed"):
            await pc.close(); _pcs.discard(pc)

    await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
    pc.addTrack(SlotVideoTrack(slot))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return pc.localDescription.sdp


def handle_offer_sync(slot: int, sdp: str, timeout: float = 10.0) -> str:
    if _loop is None:
        init_webrtc(_get_latest_frame or (lambda s: None))
    fut = asyncio.run_coroutine_threadsafe(_handle_offer_async(slot, sdp), _loop)
    return fut.result(timeout=timeout)

