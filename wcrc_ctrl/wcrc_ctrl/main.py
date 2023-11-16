import rclpy
from rclpy.node import Node
from .Wcrc import Wcrc

# cmd_vel
from geometry_msgs.msg import Twist


def main(args=None):
    rclpy.init(args=args)
    node = Node('wcrc')
    wcrc = Wcrc(node)
    
    while not rclpy.ok():
        rclpy.spin_once(node)
        wcrc.control()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()