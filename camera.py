import threading, time, glob, subprocess, re, os
from typing import Dict, List
import cv2


class CameraManager:
    def __init__(self):
        self.cap = None
        self.lock = threading.Lock()
        self.applied = {
            "device": None,
            "fourcc": None,
            "width": None,
            "height": None,
            "fps": None
        }
        self._read_thread = None
        self._stop = threading.Event()
        self._latest = None

    def _reader_loop(self):
        while not self._stop.is_set():
            with self.lock:
                cap = self.cap
            if cap is None:
                time.sleep(0.02)
                continue
            ok, frame = cap.read()
            if ok:
                self._latest = frame
            else:
                time.sleep(0.005)

    def _open(self, device, fourcc, width, height, fps):
        if self.cap is not None:
            try:
                self.cap.release()
            except:
                pass
            self.cap = None

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"เปิดกล้องไม่ได้: {device}")

        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        if width and height:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        if fps:
            cap.set(cv2.CAP_PROP_FPS, float(fps))

        time.sleep(0.2)

        got_w  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h  = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        got_fps = float(cap.get(cv2.CAP_PROP_FPS))
        got_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
        got_fourcc_str = "".join([chr((got_fourcc >> 8*i) & 0xFF) for i in range(4)])

        ok, _ = cap.read()
        if not ok:
            cap.release()
            raise RuntimeError("อ่านภาพไม่ได้หลังตั้งค่า (format อาจไม่ตรง/USB ไม่พอ)")

        self.cap = cap
        self.applied.update({
            "device": device,
            "fourcc": got_fourcc_str,
            "width": got_w,
            "height": got_h,
            "fps": round(got_fps, 2)
        })

        # (re)start reader thread
        self._stop.clear()
        if self._read_thread is None or not self._read_thread.is_alive():
            self._read_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._read_thread.start()

    def apply(self, device, fourcc, width, height, fps):
        with self.lock:
            self._open(device, fourcc, width, height, fps)
            return dict(self.applied)

    def frames(self):
        while True:
            frame = self._latest
            if frame is None:
                time.sleep(0.02)
                continue
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                time.sleep(0.01)
                continue
            yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")

    def latest_frame(self):
        return self._latest


cam_mgrs = {1: CameraManager(), 2: CameraManager(), 3: CameraManager()}


def list_video_devices() -> List[Dict]:
    devs = []
    for path in sorted(glob.glob("/dev/video*")):
        name = None
        try:
            out = subprocess.check_output(["v4l2-ctl", "-d", path, "--all"],
                                          stderr=subprocess.STDOUT,
                                          text=True)
            m = re.search(r"Card type\s*:\s*(.+)", out)
            if m: name = m.group(1).strip()
        except Exception:
            pass
        devs.append({"device": path, "name": name or os.path.basename(path)})
    return devs


def list_caps(device: str) -> Dict:
    from typing import Dict, List
    caps: Dict[str, Dict[str, List[float]]] = {}
    try:
        out = subprocess.check_output(
            ["v4l2-ctl", "-d", device, "--list-formats-ext"],
            stderr=subprocess.STDOUT, text=True
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(e.output)

    current_fmt = None
    current_size = None

    def add_fps(fmt, size, fps_val: float):
        if not fmt or not size:
            return
        f = round(float(fps_val), 1)
        arr = caps[fmt].setdefault(size, [])
        for x in arr:
            if abs(x - f) < 0.05:
                return
        arr.append(f)

    for line in out.splitlines():
        line = line.strip()
        m = re.match(r"\[\d+\]: '([A-Z0-9]{4})'\s*\((.+)\)", line)
        if m:
            current_fmt = m.group(1)
            caps.setdefault(current_fmt, {})
            current_size = None
            continue
        m = re.match(r"Size:\s+Discrete\s+(\d+)x(\d+)", line)
        if m and current_fmt:
            current_size = f"{m.group(1)}x{m.group(2)}"
            caps[current_fmt].setdefault(current_size, [])
            continue
        m = re.match(r"Size:\s+Stepwise\s+(\d+)x(\d+)\s+to\s+(\d+)x(\d+)", line)
        if m and current_fmt:
            min_w, min_h = m.group(1), m.group(2)
            current_size = f"{min_w}x{min_h}"
            caps[current_fmt].setdefault(current_size, [])
            continue
        m = re.search(r"\(([\d\.]+)\s*fps\)", line)
        if m and current_fmt and current_size:
            fps = float(m.group(1))
            add_fps(current_fmt, current_size, fps)
            continue
        m = re.search(r"framerate\s*:\s*(\d+)\s*/\s*(\d+)", line, re.IGNORECASE)
        if m and current_fmt and current_size:
            num, den = int(m.group(1)), int(m.group(2))
            if den != 0:
                fps = num/den
                add_fps(current_fmt, current_size, fps)
            continue

    for fmt in caps:
        for sz in caps[fmt]:
            caps[fmt][sz] = sorted(caps[fmt][sz])
    return caps


def boot_default(DEFAULT_DEV: str = "/dev/video0"):
    devs = list_video_devices()
    devices = [d["device"] for d in devs]
    for slot in (1, 2, 3):
        dev = devices[slot-1] if len(devices) >= slot else (devices[0] if devices else DEFAULT_DEV)
        try:
            caps = list_caps(dev)
            prefer_order = ["MJPG", "YUYV"]
            chosen = None
            for fmt in prefer_order + list(caps.keys()):
                if fmt in caps:
                    sizes = sorted(caps[fmt].keys(), key=lambda s: (int(s.split('x')[0]) * int(s.split('x')[1])))
                    for sz in sizes:
                        fps_list = caps[fmt][sz]
                        if fps_list:
                            w, h = map(int, sz.split("x"))
                            f_sel = min(fps_list, key=lambda x: abs(x-20.0))
                            chosen = (fmt, w, h, f_sel)
                            break
                if chosen: break
            if chosen:
                fmt, w, h, fps = chosen
                cam_mgrs[slot].apply(dev, fmt, w, h, fps)
        except Exception as e:
            print(f"[WARN] boot slot {slot} failed:", e)

