import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

# cmd_vel
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Sensor():
    def __init__(self, node):
        self.node = node
        self.node.get_logger().info('init sensor')
        
        # sub odom
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_odom = self.node.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            qos_profile)
        self.odom_msg = None
        # sub camera
        self.sub_camera = self.node.create_subscription(
            Image,
        '/camera/image_raw',
            self.camera_callback,
            10) 
        self.camera_msg = None
        
        self.sub_string = self.node.create_subscription(
            String,
            'command',
            self.command_callback,
            10)
    def odom_callback(self, msg):
        print("odom_callback")
        self.odom_msg = msg
    
    def camera_callback(self, msg):
        self.camera_msg = msg
    
    def command_callback(self, msg):
        self.node.get_logger().info('command_callback: "%s"' % msg.data)
        self.command = msg.data
    
    def init(self):
        # ready sensor
        self.node.get_logger().info('wait sensor')
        while self.odom_msg is None:
            pass
            
        # while self.camera_msg is None:
        #     pass
        self.node.get_logger().info('sensor ready!')