"""
Module: ros_wrapper.py
Handles: Optional ROS 2 integration
Owns: Publisher setup, ROS node initialization
Calls: rclpy
Does not contain: detection, tracking, image processing, or logic
"""

import struct
from config import LOG


try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    USE_ROS = True
except ImportError:
    USE_ROS = False

    class String:
        def __init__(self):
            self.data = ""

    class Node:
        def __init__(self, name: str):
            pass

        def create_publisher(self, *args, **kwargs):
            return None

        def destroy_node(self):
            pass

    class rclpy:
        @staticmethod
        def init():
            pass

        @staticmethod
        def shutdown():
            pass


_SHAPE_IDX = {"circle": 0, "square": 1}
_COLOR_IDX = {"red": 0, "blue": 1, "green": 2, "yellow": 3}


class ROSInterface:
    def __init__(self, node_name="perception_controller"):
        self.enabled = USE_ROS
        self.node = Node(node_name) if USE_ROS else None
        self.publisher = (
            self.node.create_publisher(String, "object_data", 10) if self.node else None
        )

    def publish(self, shape: str, color: str, tid: int, x: int, y: int, z: int):
        if not self.enabled or self.publisher is None:
            LOG.info(f"[SEND] id={tid} shape={shape} color={color} xyz=({x},{y},{z})")
            return

        xyz_i16 = (int(round(x)), int(round(y)), int(round(z)))
        payload = struct.pack(
            "<H B B h h h B",
            tid,
            _SHAPE_IDX.get(shape, 255),
            _COLOR_IDX.get(color, 255),
            *xyz_i16,
        )
        msg = String()
        msg.data = payload.hex()
        self.publisher.publish(msg)

    def destroy(self):
        if self.node and self.enabled:
            self.node.destroy_node()


def ros_init():
    if USE_ROS:
        rclpy.init()


def ros_shutdown():
    if USE_ROS:
        rclpy.shutdown()
