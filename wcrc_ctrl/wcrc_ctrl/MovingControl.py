# position control using PID
from typing import Any
import rclpy as rp
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String
# sub odom
from nav_msgs.msg import Odometry
import math
import transformations
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import numpy as np



class MovingControl(Node):
    def __init__(self):
        super().__init__('position_control')

        print("start init")
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, qos_profile)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_msg = Twist()

        # sub command
        self.command_sub = self.create_subscription(
            String, 'command', self.command_callback, 10)

        self.odom_msg = Odometry()
        # 초기값 설정

        # 현재 위치와 각도 정보
        self.current_pos = 0  # 현재 위치(미사용)
        self.current_pos_x = 0.0  # 현재 X 좌표
        self.current_pos_y = 0.0  # 현재 Y 좌표
        self.current_theta = 0  # 현재 각도 (yaw)

        # 제어에 사용되는 변수
        self.lastError = 0  # PID 제어를 위한 이전 오차
        self.last_current_theta = 0.0  # 이전의 현재 각도
        self.last_current_pos = 0.0  # 이전의 현재 위치 (미사용)

        # 이동 명령 관련 변수
        self.cmd_moving = None  # 이동 명령 (앞으로 이동, 뒤로 이동, 등)

        # 이동 명령을 수행하기 위한 목표 거리
        self.target_distance = 1.  # 예시로 설정된 목표 이동 거리

        # 현재 이동한 거리
        self.current_distance = 0.0

        # 목표 각도와 현재 각도
        self.target_theta = np.deg2rad(91.25)   # 예시로 설정된 목표 각도 (π/2 라디안)
        self.current_theta = 0.0  # 현재 각도

        # 이동 시작 위치 및 방향 정보
        self.start_x = 0.0  # 이동 시작 X 좌표
        self.start_y = 0.0  # 이동 시작 Y 좌표
        self.start_theta = 0.0  # 이동 시작 각도 (yaw)

        # 원하는 각도 정보 (예시로 설정된 값)
        self.desired_theta = 0.0

        # control flag
        self.is_step_start = False
        # 허용 오차 범위
        self.error_range = 0.04

        self.info("start init")
        # timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialize class variables or state parameters here if needed.
        self.control_dict = {
            # 일정거리 직진
            'go': self.go_straight,
            # 정지
            'stop': self.stop,
            # 좌회전
            'left': self.left_turn,
            # 우회전
            'right': self.right_turn,
            # 후진
            'back': self.back,
        }

        self.cmd_moving = None
        self.flag = False
    def command_callback(self, msg):
        self.cmd_moving = msg.data
        # 올바른 입력이 아닐 경우
        if self.cmd_moving not in self.control_dict.keys():
            self.error("Wrong command")
        else:
            if self.flag == False: 
                self.control_dict[self.cmd_moving]()
                self.info("Command: " + self.cmd_moving)
                self.flag = True

    # 시작 flag와 완료 flag를 이용하여 명령 수행

    def timer_callback(self):
        if self.cmd_moving == None or self.control_dict[self.cmd_moving]() == True:
            self.stop()
            return
        else:
            self.control_dict[self.cmd_moving]()

    def odom_callback(self, msg):
        self.odom_msg = msg
        # print("start")
        quaternion = (self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y,
                      self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w)
        # quaternion to euler
        euler_angles = self.euler_from_quaternion(quaternion)
        # Extracting only the yaw component
        self.current_theta = euler_angles[0]
        # self.info("current_theta: " + str(self.current_theta))

        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        self.current_pos_x = self.odom_msg.pose.pose.position.x
        self.current_pos_y = self.odom_msg.pose.pose.position.y
        # print(self.current_pos_x)

    def euler_from_quaternion(self, quaternion):
        # Convert the Quaternion to a list [x, y, z, w]
        quaternion_list = [quaternion[0],
                           quaternion[1], quaternion[2], quaternion[3]]
        # Perform the conversion
        euler = transformations.euler_from_quaternion(quaternion_list)
        return euler  # Returns a tuple (roll, pitch, yaw)

    def drive(self, linear_velocity, angular_velocity):
        self.cmd_vel_msg.linear.x = linear_velocity
        self.cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def normalize_angle(self, angle):
        """
        Normalize the angle to be within -pi and pi.
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def left_turn(self):
        self.info("left turn")
        Kp = 0.6
        Ki = 0.0
        Kd = 0.05
        error = 10.0
        if self.is_step_start == False:
            self.disired_theta = self.normalize_angle(
                self.current_theta + self.target_theta)
            self.is_step_start = True
            self.lastError = 0

        error = self.normalize_angle(self.current_theta - self.disired_theta)
        control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error

        if math.fabs(error) > self.error_range - 0.0:
            self.drive(0.0, -2*control)
            print("error " + str(error), self.current_theta,
                  "       ", self.disired_theta)
        else:
            self.drive(0.0, 0.0)
            self.is_step_start = False
            self.info("left turn end")
            self.cmd_moving = None
            self.flag = False
            return True
        return False

    def right_turn(self):
        self.info("right turn")
        Kp = 0.6
        Ki = 0.0
        Kd = 0.05
        error = 10.0
        if self.is_step_start == False:
            self.disired_theta = self.normalize_angle(
                self.current_theta - self.target_theta)
            self.is_step_start = True
            self.lastError = 0

        error = self.normalize_angle(self.current_theta - self.disired_theta)
        control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error

        if math.fabs(error) > self.error_range - 0.0:
            self.drive(0.0, -2*control)
            # print("error ", error)
            print("error " + str(error), self.current_theta,
                  "       ", self.disired_theta)
        else:
            self.drive(0.0, 0.0)
            self.is_step_start = False
            self.info("right turn end")
            self.cmd_moving = None
            self.flag = False
            return True
        return False

    def stop(self):
        # self.info("stop")
        self.drive(0.0, 0.0)
        return True

    def back(self):
        self.info("back")
        Kp = 0.8
        Ki = 0.0
        Kd = 0.05
        error = 10.0
        if self.is_step_start == False:
            self.start_x = self.current_pos_x
            self.start_y = self.current_pos_y
            self.is_step_start = True
            self.lastError = 0

        error = math.sqrt((self.current_pos_x - self.start_x)**2 +
                          (self.current_pos_y - self.start_y)**2) - self.target_distance
        control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error

        if math.fabs(error) > self.error_range:
            self.drive(2*control, 0.0)
            print("error ", error)
        else:
            self.drive(0.0, 0.0)
            self.is_step_start = False
            self.info("back end")
            self.cmd_moving = None
            self.flag = False
            return True
        return False

    def go_straight(self):
        self.info("go")
        Kp = 0.8
        Ki = 0.0
        Kd = 0.05
        error = 10.0

        if self.is_step_start == False:
            self.start_x = self.current_pos_x
            self.start_y = self.current_pos_y
            self.is_step_start = True
            self.lastError = 0

        error = math.sqrt((self.current_pos_x - self.start_x)**2 +
                          (self.current_pos_y - self.start_y)**2) - self.target_distance
        control = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error

        if math.fabs(error) > self.error_range:
            self.drive(-2*control, 0.0)
            print("error ", error)
        else:
            self.drive(0.0, 0.0)
            self.is_step_start = False
            self.info("go end")
            self.cmd_moving = None
            self.flag = False
            return True
        return False

    def info(self, msg):
        self.get_logger().info(msg)

    def warn(self, msg):
        self.get_logger().warn(msg)

    def error(self, msg):
        self.get_logger().error(msg)


    def __call__(self, mode):
        self.info("start")
        self.cmd_moving = mode
        self.flag = False
        self.timer_callback()
        self.info("end")

def main(args=None):
    rp.init(args=args)
    wcrc = MovingControl()
    
    rp.spin(wcrc)


if __name__ == '__main__':
    main()