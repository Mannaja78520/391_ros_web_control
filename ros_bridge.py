import time
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as RosTime
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray, String
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
            "data": {"data": arr, "layout": {"dim": [], "data_offset": 0}},
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
                "header": {"frame_id": msg.header.frame_id, "child_frame_id": msg.child_frame_id},
                "pose": {"pose": {"position": {"x": p.x, "y": p.y, "z": p.z}, "orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w}}},
                "twist": {"twist": {"linear": {"x": v.linear.x, "y": v.linear.y, "z": v.linear.z}, "angular": {"x": v.angular.x, "y": v.angular.y, "z": v.angular.z}}}
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
            "data": {"orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w}},
            "t": t
        }
        self.safe_push(payload)

    def on_status(self, msg: String):
        payload = {"topic": "/status", "type": "std_msgs/String", "data": {"status": msg.data}, "t": time.time()}
        self.safe_push(payload)


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

    import threading
    t = threading.Thread(target=spin, daemon=True)
    t.start()

