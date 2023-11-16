
import rclpy
from rclpy.node import Node

# extention sensor
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import numpy as np

from .MovingControl import MovingControl
from .Sensor import Sensor



class Wcrc():
      def __init__(self, node):
          # super().__init__('wcrc')
          self.node = node
          self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
          self.cmd_vel_msg = Twist()
          
          self.sensor = Sensor(self.node)
          self.sensor.init()
           
      def control(self):
          # print("start control")
          print(self.sensor.odom_msg)
      
    