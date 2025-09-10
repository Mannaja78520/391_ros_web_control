#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import threading, time, json, asyncio

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray, String

class RosBridge(Node):
    def __init__(self, push_fn):
        super().__init__('ros_bridge')
        self.push = push_fn

        # subscribe topics ที่ต้องการ
        self.create_subscription(Int32MultiArray, '/encoder', self.on_encoder, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, 10)
        self.create_subscription(String, '/status', self.on_status, 10)

    def on_encoder(self, msg: Int32MultiArray):
        # สมมติ msg.data = [left, right]
        payload = {
            "topic": "/encoder",
            "type": "std_msgs/Int32MultiArray",
            "data": {
                "left": int(msg.data[0]) if len(msg.data) > 0 else 0,
                "right": int(msg.data[1]) if len(msg.data) > 1 else 0
            },
            "t": time.time()
        }
        self.push(payload)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        payload = {
            "topic": "/odom",
            "type": "nav_msgs/Odometry",
            "data": {
                "pose": {
                    "pose": {
                        "position": {"x": p.x, "y": p.y, "z": p.z},
                        "orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w}
                    }
                }
            },
            "t": time.time()
        }
        self.push(payload)

    def on_imu(self, msg: Imu):
        o = msg.orientation
        payload = {
            "topic": "/imu",
            "type": "sensor_msgs/Imu",
            "data": {
                "orientation": {"x": o.x, "y": o.y, "z": o.z, "w": o.w}
            },
            "t": time.time()
        }
        self.push(payload)
        
    def on_status(self, msg: String):
        # สมมติ msg.data = [left, right]
        payload = {
            "topic": "/status",
            "type": "std_msgs/String",
            "data": {
                "status": msg.data
            },
            "t": time.time()
        }
        self.push(payload)
    

# =======================
# Run ROS2 in background
# =======================
def start_ros(push_fn):
    rclpy.init()
    node = RosBridge(push_fn)

    def spin():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    t = threading.Thread(target=spin, daemon=True)
    t.start()
    return node
