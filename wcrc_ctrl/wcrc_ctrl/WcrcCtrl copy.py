
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

from .Sensor import Sensor
from .Logger import Logger
from .MovingCtrl import MovingCtrl


class WcrcCtrl:
    def __init__(self, node: Node):
        # Ensure that the node argument is indeed an instance of rclpy.node.Node
        if not isinstance(node, Node):
            raise TypeError("Logger expects an rclpy.node.Node instance.")
        self.node = node
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_msg = Twist()
        self.logger = Logger(self.node)
        self.sensor = Sensor(self.node)
        self.moving_ctrl = MovingCtrl(self.node)

        self.detect_node = self.node.create_subscription(
            String, "/detect", self.detect_callback, 10
        )
        self.current_node = None

        self.init_flag = False

        self.logger.info("wcrc_ctrl started")
        self.logger.info("wcrc_ctrl sensor wait...")

        self.mode = "left"
        self.cnt = 0
        # dfs를 이용한 맵 생성
        self.map = np.zeros((10, 10))
        self.map[0][0] = 1

        self.x, self.y = 0, 0
        self.dir = 0
        self.visited = np.zeros((10, 10))
        self.visited[0][0] = 1
        # 재귀로 구현

        # joy 동남서북
        # 왼쪽 아래 오른쪽 위
        # 오른쪽 방향으로 90도 회전
        self.dy = [-1, 0, 1, 0]
        self.dx = [0, 1, 0, -1]

        self.queue = []

        # 노드가 있는곳을 1로 표시

        self.control_dict = {
            "right": self.right,
            "forward": self.forward,
            "dfs": self.dfs,
            "terminate": self.terminate,
        }

        self.forward_flag = False
        self.right_flag = False

        self.right_cnt = 0

    def detect_callback(self, msg):
        self.current_node = msg.data

    def dfs(self, y, x):
        self.logger.info("dfs")

        if self.moving_ctrl("forward"):
            self.forward_flag = True
        else:
            self.moving_ctrl("forward")

        if self.forward_flag:
            # 방문
            self.visited[y][x] = 1

            # 왼쪽 오른쪽 아래 위 탐색
            for i in range(4):

                ny = y + self.dy[i]
                nx = x + self.dx[i]

                if ny < 0 or nx < 0 or ny >= 10 or nx >= 10:
                    continue
                # 노드가 갈 수 없는 곳일때
                if self.map[ny][nx] == 0:
                    continue
                # 방문 했을때
                if self.visited[ny][nx] == 1:
                    continue
                # 마지막 노드일 경우
                self.queue.append((y, x))
                self.dfs(ny, nx)

            # 더이상 갈곳이 없을때
            if self.queue:
                ny, nx = self.queue.pop()
                self.dfs(ny, nx)
            else:
                self.mode = "terminate"
                self.terminate()

    def detect(self):
        if self.current_node == "detect":
            self.logger.info("detect")
            self.queue.append("forward")
            self.queue.pop()
            self.mode = self.queue[0]

    def right(self, *args):
        # 성공할 때까지 오른쪽으로 회전을 시도합니다.
        if not self.moving_ctrl("right"):
            self.moving_ctrl("right")  # 다시 오른쪽으로 회전을 시도합니다.
        else:
            # 회전이 완료되면, 다음 명령을 실행하거나 멈춥니다.
            self.mode = self.queue[0] if self.queue else "정지"

    def forward(self, *args):
        # 감지하거나 완료될 때까지 전진을 계속합니다.
        if not self.moving_ctrl("forward"):
            self.moving_ctrl("forward")  # 다시 전진을 시도합니다.
        else:
            # 이동이 완료되면, 다음 명령을 실행하거나 멈춥니다.
            self.mode = self.queue[0] if self.queue else "정지"

    def terminate(self):
        self.logger.info("terminate")
        self.moving_ctrl("stop")

    def control(self):
        # 로봇이 초기화되었는지 및 센서 데이터가 있는지 확인합니다.
        if not self.init_flag and self.sensor.odom_msg is not None:
            self.logger.info("시작")
            self.init_flag = True

        # 큐에서 명령을 처리합니다.
        if self.queue and self.init_flag:
            command, *args = self.queue.pop(0)  # 큐에서 다음 명령을 꺼냅니다.
            if command in self.control_dict:
                self.control_dict[command](*args)  # 명령을 실행합니다.
            else:
                self.logger.error(f"알 수 없는 명령: {command}")

        # 큐에 명령이 없고 초기화가 완료되었으며 현재 위치가 알려진 경우 DFS를 시작합니다.
        elif not self.queue and self.init_flag and self.current_node is not None:
            self.dfs(self.y, self.x)
