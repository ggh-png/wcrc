import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from wcrc_ctrl.Logger import Logger
import time


class Sensor:
    def __init__(self, node: Node):
        # Ensure that the node argument is indeed an instance of rclpy.node.Node
        if not isinstance(node, Node):
            raise TypeError("Logger expects an rclpy.node.Node instance.")
        self.node = node
        self.logger = Logger(self.node)
        # Subscriptions
        qos_profile = QoSProfile(
            depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.sub_odom = self.node.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.sub_camera = self.node.create_subscription(
            Image, '/camera2/color/image_raw', self.camera_callback, qos_profile)

        # Sensor data
        self.odom_msg = None
        self.camera_msg = None
        self.command = None

    def odom_callback(self, msg):
        # self.node.get_logger().info('Odometry callback triggered')
        self.odom_msg = msg

    def camera_callback(self, msg):
        # self.node.get_logger().info('Camera callback triggered')
        self.camera_msg = msg

    def init(self):
        self.logger.info("wcrc_ctrl sensor wait...")
        if self.odom_msg is not None and self.camera_msg is not None:
            return True
        else:
            return False
