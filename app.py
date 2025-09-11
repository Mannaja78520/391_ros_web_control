#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re, os, glob, subprocess, threading, time, json
from typing import Dict, List

from flask import Flask, jsonify, request, Response, send_from_directory
from flask_sock import Sock
import cv2

# ============================
# Flask app
# ============================
APP_HOST = "0.0.0.0"
APP_PORT = 8000
DEFAULT_DEV = "/dev/video0"

app = Flask(__name__, static_url_path="/static")
sock = Sock(app)

# ============================
# WebSocket broadcast (ROS push)
# ============================
clients = set()
clients_lock = threading.Lock()

def ws_broadcast(msg_dict):
    data = json.dumps(msg_dict, ensure_ascii=False)
    with clients_lock:
        dead = []
        for ws in list(clients):
            try:
                ws.send(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            clients.discard(ws)

@sock.route('/ws')
def ws(ws):
    # สมัครรับข้อความ ROS แบบ realtime (server -> client)
    with clients_lock:
        clients.add(ws)
    try:
        # รับข้อความจาก client ถ้ามี (ไม่ใช้ ก็แค่คงการเชื่อมต่อไว้)
        while True:
            _ = ws.receive()
            if _ is None:
                break
    except Exception:
        pass
    finally:
        with clients_lock:
            clients.discard(ws)

# ============================
# Camera Manager (multi-slot)
# ============================
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

        # ลำดับสำคัญ: FOURCC -> W/H -> FPS
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
        print("Applied:", self.applied)

    def apply(self, device, fourcc, width, height, fps):
        with self.lock:
            self._open(device, fourcc, width, height, fps)
            return dict(self.applied)

    def frames(self):
        while True:
            with self.lock:
                cap = self.cap
            if cap is None:
                time.sleep(0.05)
                continue
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                continue
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" +
                   jpg.tobytes() + b"\r\n")

# 3 ช่องกล้อง (slot 1..3)
cam_mgrs = {1: CameraManager(), 2: CameraManager(), 3: CameraManager()}

# ============================
# Helpers: list devices & caps
# ============================

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
        m = re.search(r"\(([\d\.]+)\s*fps\)", line)
        if m and current_fmt and current_size:
            fps = round(float(m.group(1)))
            if fps not in caps[current_fmt][current_size]:
                caps[current_fmt][current_size].append(fps)
            continue
        m = re.search(r"framerate\s*:\s*(\d+)\s*/\s*(\d+)", line, re.IGNORECASE)
        if m and current_fmt and current_size:
            num, den = int(m.group(1)), int(m.group(2))
            if den != 0:
                fps = round(num/den)
                if fps not in caps[current_fmt][current_size]:
                    caps[current_fmt][current_size].append(fps)
            continue

    for fmt in caps:
        for sz in caps[fmt]:
            caps[fmt][sz] = sorted(caps[fmt][sz], reverse=True)
    return caps

# ============================
# Routes (API + streams)
# ============================
@app.route("/")
def index():
    return send_from_directory("static", "index.html")

@app.get("/api/devices")
def api_devices():
    return jsonify({"devices": list_video_devices()})

@app.get("/api/caps")
def api_caps():
    dev = request.args.get("dev", DEFAULT_DEV)
    try:
        return jsonify({"device": dev, "caps": list_caps(dev)})
    except Exception as e:
        return jsonify({"error": str(e)}), 400

@app.post("/api/apply")
def api_apply():
    data = request.get_json(silent=True) or {}
    slot = int(data.get("slot", 1))
    device = data.get("device", DEFAULT_DEV)
    fourcc = (data.get("fourcc", "MJPG") or "MJPG").upper()
    width, height = data.get("width"), data.get("height")
    fps = data.get("fps")

    if slot not in cam_mgrs:
        return jsonify({"error": "slot ต้องเป็น 1..3"}), 400

    try:
        width = int(width); height = int(height); fps = int(fps)
    except Exception:
        return jsonify({"error": "width/height/fps ต้องเป็นตัวเลข"}), 400

    if fourcc not in ("MJPG", "YUYV", "YUY2", "H264", "XVID"):
        return jsonify({"error": f"FOURCC '{fourcc}' ไม่ถูกสนับสนุน"}), 400

    try:
        applied = cam_mgrs[slot].apply(device, fourcc, width, height, fps)
        return jsonify({"slot": slot, "applied": applied})
    except Exception as e:
        return jsonify({"error": str(e)}), 400

@app.get("/api/applied")
def api_applied():
    slot = int(request.args.get("slot", 1))
    if slot not in cam_mgrs:
        return jsonify({"error": "slot ต้องเป็น 1..3"}), 400
    return jsonify({"slot": slot, "applied": cam_mgrs[slot].applied})

@app.get("/video/<int:slot>.mjpg")
def video_mjpg(slot: int):
    if slot not in cam_mgrs:
        return Response(status=404)
    return Response(cam_mgrs[slot].frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# ============================
# ROS bridge (subscribe & push to WS)
# ============================
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as RosTime
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout, String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

def ros_time_to_float(t: RosTime):
    return float(t.sec) + float(t.nanosec) * 1e-9

class RosBridge(Node):
    def __init__(self, push_fn):
        super().__init__('ros_bridge')
        self.push = push_fn
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.create_subscription(Int32MultiArray, '/encoder', self.on_encoder, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, sensor_qos)
        self.create_subscription(Imu, '/imu', self.on_imu, sensor_qos)
        self.create_subscription(String, '/status', self.on_status, 10)

    def safe_push(self, payload):
        try:
            self.push(payload)
        except Exception as e:
            print(f"[RosBridge] push error: {e}")

    def on_encoder(self, msg: Int32MultiArray):
        arr = list(msg.data) if msg.data is not None else []
        payload = {
            "topic": "/encoder",
            "type": "std_msgs/Int32MultiArray",
            "data": {
                "data": arr,
                "layout": {
                    "dim": [
                        {"label": d.label, "size": d.size, "stride": d.stride}
                        for d in msg.layout.dim
                    ],
                    "data_offset": msg.layout.data_offset
                }
            },
            "t": time.time()
        }
        self.safe_push(payload)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        v = msg.twist.twist
        t = ros_time_to_float(msg.header.stamp) if msg.header.stamp else time.time()
        payload = {
            "topic": "/odom",
            "type": "nav_msgs/Odometry",
            "data": {
                "header": {
                    "frame_id": msg.header.frame_id,
                    "child_frame_id": msg.child_frame_id
                },
                "pose": {
                    "pose": {
                        "position": {"x": p.x, "y": p.y, "z": p.z},
                        "orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w}
                    }
                },
                "twist": {
                    "twist": {
                        "linear": {"x": v.linear.x, "y": v.linear.y, "z": v.linear.z},
                        "angular": {"x": v.angular.x, "y": v.angular.y, "z": v.angular.z}
                    }
                }
            },
            "t": t
        }
        self.safe_push(payload)

    def on_imu(self, msg: Imu):
        o = msg.orientation
        t = ros_time_to_float(msg.header.stamp) if msg.header.stamp else time.time()
        payload = {
            "topic": "/imu",
            "type": "sensor_msgs/Imu",
            "data": {
                "orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w}
            },
            "t": t
        }
        self.safe_push(payload)

    def on_status(self, msg: String):
        payload = {
            "topic": "/status",
            "type": "std_msgs/String",
            "data": {"status": msg.data},
            "t": time.time()
        }
        self.safe_push(payload)

# Thread: rclpy.spin
_ros_node = None

def start_ros(push_fn):
    global _ros_node
    rclpy.init()
    _ros_node = RosBridge(push_fn)

    def spin():
        try:
            rclpy.spin(_ros_node)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                _ros_node.destroy_node()
            finally:
                rclpy.shutdown()

    t = threading.Thread(target=spin, daemon=True)
    t.start()

# ============================
# Boot defaults
# ============================

def boot_default():
    devs = list_video_devices()
    devices = [d["device"] for d in devs]
    # map อุปกรณ์ 3 ตัวแรก -> slot 1..3 ถ้ามีน้อยกว่านั้นก็ซ้ำตัวแรก
    for slot in (1,2,3):
        dev = devices[slot-1] if len(devices) >= slot else (devices[0] if devices else DEFAULT_DEV)
        try:
            caps = list_caps(dev)
            # เลือก MJPG > YUYV
            prefer_order = ["MJPG", "YUYV"]
            chosen = None
            for fmt in prefer_order + list(caps.keys()):
                if fmt in caps:
                    sizes = sorted(caps[fmt].keys(),
                                   key=lambda s: (int(s.split('x')[0])*int(s.split('x')[1])),
                                   reverse=True)
                    for sz in sizes:
                        fps_list = caps[fmt][sz]
                        if fps_list:
                            w, h = map(int, sz.split("x"))
                            chosen = (fmt, w, h, fps_list[0])
                            break
                if chosen: break
            if chosen:
                fmt, w, h, fps = chosen
                cam_mgrs[slot].apply(dev, fmt, w, h, fps)
        except Exception as e:
            print(f"[WARN] boot slot {slot} failed:", e)

if __name__ == "__main__":
    # start ROS push -> WS clients
    start_ros(ws_broadcast)
    # open default 3 streams
    boot_default()
    app.run(host=APP_HOST, port=APP_PORT, threaded=True)