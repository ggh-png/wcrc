import rclpy
from rclpy.node import Node
from .WcrcCtrl import WcrcCtrl
from .Logger import Logger

# cmd_vel
from geometry_msgs.msg import Twist


class Wcrc(Node):
    def __init__(self):
        super().__init__('wcrc_node')
        self.logger = Logger(self)
        self.timer = self.create_timer(0.1, self.control)
        self.wcrc_ctrl = WcrcCtrl(self)
        self.logger.info("wcrc_node started")
        self.wcrc_ctrl.mode = "direction"

        # self.wcrc.right()

    def control(self):
        self.wcrc_ctrl.control()


def main(args=None):
    rclpy.init(args=args)
    wcrc = Wcrc()
    rclpy.spin(wcrc)
    wcrc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
