# ros_wrapper.py

"""
ROS import guarded by a try/except. Provides a fallback if ROS is not installed.
Handles all ROS 2 functionality for perception system in a modular way.
"""

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
    class DummyPublisher:
        def __init__(self):
            pass
        def publish(self, msg):
            pass
    class Node:
        def __init__(self, name: str):
            pass
        def create_publisher(self, *args, **kwargs):
            return DummyPublisher()
        def destroy_node(self):
            pass
    class rclpy:
        @staticmethod
        def init(*args, **kwargs):
            pass
        @staticmethod
        def spin(node):
            pass
        @staticmethod
        def shutdown():
            pass
        @staticmethod
        def ok():
            return True

__all__ = ["USE_ROS", "Node", "String", "rclpy"]


# --- Modular ROS publishing interface (move logic out of perception_controller!) ---

class ROSInterface:
    """
    Encapsulates all ROS communication for the perception node.
    Publishes String messages in scientific format:
    "shape=sphere color=Red x=123 y=456 z=789"
    """
    def __init__(self, node_name="perception_controller", pi_mode=False):
        self.pi_mode = pi_mode
        self.ros_node = None
        self.publisher = None
        if not USE_ROS or self.pi_mode:
            return
        self.ros_node = Node(node_name)
        self.publisher = self.ros_node.create_publisher(String, 'object_data', 10)

    def publish_detection(self, color:str, shape:str, x:int, y:int, z:int):
        if self.pi_mode or not USE_ROS or self.publisher is None:
            return
        msg = String()
        msg.data = f"shape={shape} color={color} x={x} y={y} z={z}"
        self.publisher.publish(msg)

    def destroy(self):
        if self.ros_node and USE_ROS:
            self.ros_node.destroy_node()

# ROS lifecycle helpers (for main loop)
def ros_init():
    if USE_ROS:
        rclpy.init()

def ros_spin(node):
    if USE_ROS:
        rclpy.spin(node)

def ros_shutdown():
    if USE_ROS:
        rclpy.shutdown()
