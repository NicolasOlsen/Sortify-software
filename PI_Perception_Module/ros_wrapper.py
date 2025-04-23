"""
ROS import guarded by a try/except. Provides a fallback if ROS is not installed.
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