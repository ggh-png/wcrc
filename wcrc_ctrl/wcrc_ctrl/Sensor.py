import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from wcrc_ctrl.Logger import Logger
import time
from cv_bridge import CvBridge, CvBridgeError


from wcrc_ctrl.LaneDetector import LaneDetector

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
        # self.sub_odom = self.node.create_subscription(
        #     Odometry, '/odom', self.odom_callback, 10)
        # self.sub_camera = self.node.create_subscription(
        #     Image, '/camera/image_raw', self.camera_callback, 10)

        self.sub_odom = self.node.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.sub_camera = self.node.create_subscription(
            Image, '/camera2/color/image_raw', self.camera_callback, 5)

        self.sub_shape = self.node.create_subscription(
            String, 'shape', self.shape_callback, 10)

        self.lane_detector = LaneDetector(self.node)


        # Sensor data
        self.odom_msg = None
        self.camera_msg = None
        self.cv_bridge = CvBridge()
        self.command = None
        self.detected_lane = None
        self.shape = None

    def shape_callback(self, msg):
        self.shape = msg.data
        # self.logger.warn(self.shape)

    def odom_callback(self, msg):
        # self.node.get_logger().info('Odometry callback triggered')
        self.odom_msg = msg

    def camera_callback(self, msg):
        # Convert the ROS Image message to OpenCV format
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            self.camera_msg = cv_image  # Assign the converted image to camera_msg
        except CvBridgeError as e:
            self.logger.error('Failed to convert image: %s' % str(e))
            
        self.detected_lane = self.lane_detector(self.camera_msg)
        # self.logger.warn(str(self.detected_lane))

    def init(self):
        # self.logger.info("wcrc_ctrl sensor wait...")
        if self.odom_msg is not None and self.camera_msg is not None and self.detected_lane is not None and self.shape is not None:
            return True
        else:
            return False
