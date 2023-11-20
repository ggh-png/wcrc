
import rclpy
from rclpy.node import Node


class Logger:
    def __init__(self, node: Node):
        # Ensure that the node argument is indeed an instance of rclpy.node.Node
        if not isinstance(node, Node):
            raise TypeError("Logger expects an rclpy.node.Node instance.")
        self.node = node

    def info(self, msg):
        # Use get_logger() method to access the Node's logger for info level logging
        self.node.get_logger().info(msg)

    def warn(self, msg):
        # Use get_logger() method to access the Node's logger for warn level logging
        self.node.get_logger().warn(msg)

    def error(self, msg):
        # Use get_logger() method to access the Node's logger for error level logging
        self.node.get_logger().error(msg)
