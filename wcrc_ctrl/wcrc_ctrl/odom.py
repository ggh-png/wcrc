import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

# cmd_vel
from geometry_msgs.msg import Twist
class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        print("start init!")
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile)
        # cmd_vel sub 
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        print("aa")
        self.get_logger().info('Received odom:\n' + str(msg))
    
    def cmd_vel_callback(self, msg):
        self.get_logger().info('Received cmd_vel:\n' + str(msg))

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
