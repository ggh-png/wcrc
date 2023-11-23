
import rclpy
from rclpy.node import Node

# extention sensor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

from wcrc_ctrl.Sensor import Sensor
from wcrc_ctrl.Logger import Logger
from wcrc_ctrl.MovingCtrl import MovingCtrl
# from wcrc_ctrl.LaneDetector import LaneDetector

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
        # self.lane_detector = LaneDetector(self.node)
        

        self.detect_node = self.node.create_subscription(
            String, "/detect", self.detect_callback, 10
        )
        self.current_node = 1
        self.node_name = "1"

        self.init_flag = False

        self.logger.info("wcrc_ctrl started")
        self.logger.info("wcrc_ctrl sensor wait...")

        self.mode = "direction"

        # 이전 방향을 저장하는 변수

        self.control_dict = {
            "forward":  self.forward,
            "terminate": self.terminate,
            "direction": self.search_all_directions,
            "back": self.back,
            "goback": self.goback,
            "go": self.go,
        }

        # dfs를 이용한 맵 생성 string으로 변환eeeeeㄴㅁㄴㅇㅁㄴ
        # 6 X 6 맵을 만듭니다.
        self.map = [[0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0]]
        
        # 맵의 중앙에 로봇을 놓습니다.
        # 로봇의 현재 위치를 기록합니다.

        self.stack = []
        self.map[2][2] = 1

        self.x, self.y = 2, 2
        self.dir = 0
        self.last_dir = 0
        self.current_dir = 0
        self.visited = np.zeros((10, 10))
        self.visited[2][2] = 1
        # joy 동남서북
        # 아래 오른쪽 위 왼쪽
        # 오른쪽 방향으로 90도 회전
        self.dy = [0, 1, 0, -1]
        self.dx = [1, 0, -1, 0]
        self.cnt = 0
        self.stack.append((self.y, self.x, self.dir))
        self.stack.append((self.y, self.x, self.dir))
        self.init_flag = False
        
        self.go_flag = True
        
        self.direction_check = [0, 0, 0, 0]
        self.terminate_cnt = 0
        self.terminate_turn_cnt = 0
        
        
        # 재귀로 구현
    # 0이면 노드가 없는 곳, 1이면 노드가 있는 곳

    def detect_callback(self, msg):
        self.current_node = msg.data

    # 모든 방향을 탐색하는 함수고 이 함수는 노드가 있는 방향을 리턴합니다.

    def search_all_directions(self):
        # 로봇의 현재 방향을 기준으로 모든 방향을 탐색합니다.
        # 왼쪽, 아래, 오른쪽, 순서로 탐색합니다.
        # 리턴은 0, 1, 2로 합니다.
        # self.dir = 0

        # 왼쪽 방향으로 90도 회전 후 탐색
        # self.init_flag = False
        # all direction search check list 
        # direction_check = [0, 0, 0, 0]
        self.current_node = self.sensor.detected_lane
        if self.dir == 0:
            node_present = self.current_node
            nx, ny = self.x + \
                self.dx[self.dir], self.y + \
                self.dy[self.dir]
            if node_present is not 1004   and self.visited[ny][nx] == 0:
                if self.visited[ny][nx]:
                    return
                self.visited[ny][nx] = 1
                for i in range(6):
                    for j in range(6):
                        print(self.visited[i][j], end=" ")
                    print()
                    # 복귀를 위한 부모 노드의 정보를 저장합니다.
                self.last_dir = self.dir
                self.stack.append((self.y, self.x, self.dir))
                self.y, self.x = ny, nx

                self.mode = "forward"
                return
            else:
                if not self.moving_ctrl("left"):
                    self.moving_ctrl("left")
                    return
                else:
                    self.dir += 1
                    self.direction_check[0] = 1
                    return
        # 아래쪽 방향으로 90도 회전 후 탐색
        elif self.dir == 1:
            node_present = self.current_node
            nx, ny = self.x + \
                self.dx[self.dir], self.y + \
                self.dy[self.dir]
            if node_present is not 1004   and self.visited[ny][nx] == 0:

                # 이미 방문한 곳이라면 방향을 바꿔서 탐색합니다.
                if self.visited[ny][nx]:
                    return
                self.visited[ny][nx] = 1
                for i in range(6):
                    for j in range(6):
                        print(self.visited[i][j], end=" ")
                    print()
                    # 복귀를 위한 부모 노드의 정보를 저장합니다.
                self.last_dir = self.dir
                self.stack.append((self.y, self.x, self.dir))
                self.y, self.x = ny, nx

                self.mode = "forward"
                return
            else:
                if not self.moving_ctrl("left"):
                    self.moving_ctrl("left")
                    return
                else:
                    self.dir += 1
                    self.direction_check[1] = 1
                    return

        # 오른쪽 방향으로 90도 회전 후 탐색
        elif self.dir == 2:
            node_present = self.current_node
            nx, ny = self.x + \
                self.dx[self.dir], self.y + \
                self.dy[self.dir]
            if node_present is not 1004   and self.visited[ny][nx] == 0:
                if self.visited[ny][nx]:
                    return
                self.visited[ny][nx] = 1
                for i in range(6):
                    for j in range(6):
                        print(self.visited[i][j], end=" ")
                    print()
                    # 복귀를 위한 부모 노드의 정보를 저장합니다.
                self.last_dir = self.dir
                self.stack.append((self.y, self.x, self.dir))
                self.y, self.x = ny, nx

                self.mode = "forward"
                return
            else:
                if not self.moving_ctrl("left"):
                    self.moving_ctrl("left")
                    return
                else:
                    self.dir += 1
                    self.direction_check[2] = 1
                    return

        # 노드가 없는 경우에는 왼쪽으로 회전하고 다시 탐색합니다.
        elif self.dir == 3:
            node_present = self.current_node
            nx, ny = self.x + \
                self.dx[self.dir], self.y + \
                self.dy[self.dir]
            if node_present is not 1004  and self.visited[ny][nx] == 0:
                if self.visited[ny][nx]:
                    return
                self.visited[ny][nx] = 1

                for i in range(6):
                    for j in range(6):
                        print(self.visited[i][j], end=" ")
                    print()
                    # 복귀를 위한 부모 노드의 정보를 저장합니다.
                self.last_dir = self.dir
                self.stack.append((self.y, self.x, self.dir))
                self.y, self.x = ny, nx

                self.mode = "forward"
                return
            else:
                if not self.moving_ctrl("left"):
                    self.moving_ctrl("left")
                    return
                else:
                    self.dir += 1
                    self.direction_check[3] = 1
                    return
                
                
        for i in range(4):
            if self.direction_check[i] == 0:
                self.dir = 0
                return

        # 모든 방향을 탐색했다면, 복귀합니다.
        self.init_flag = True
        self.mode = "back"
        
        return

    def forward(self):
        # 전진이 완료될떄까지 이 모드는 계속해서 호출됩니다.
        if not self.moving_ctrl("forward"):
            self.moving_ctrl("forward")
            return
        else:
            # 전진이 완료되면 dfs를 다시 시작합니다.
            # self.map[self.y][self.x] = self.node_name
            # self.dir = 0
            self.direction_check = [0, 0, 0, 0]
            self.direction_check[self.dir] = 1
            self.mode = "direction"
            return

    def goback(self):
        if self.stack == 0:
            if self.init_flag == False:
                self.mode = "direction"
                self.current_node = 1
                return

            else:
                self.mode = "direction"
                return
        # 전진이 완료될떄까지 이 모드는 계속해서 호출됩니다.
        if not self.moving_ctrl("forward"):
            self.moving_ctrl("forward")
            return
        else:
            # 전진이 완료되면 dfs를 다시 시작합니다.
            self.map[self.y][self.x] = self.node_name
            # self.dir = 0

            self.mode = "back"

    def terminate(self):
        if self.terminate_turn_cnt == 2:        
            self.logger.info("terminate")
            self.moving_ctrl("stop")
            # 맵 정보를 출력합니다.
            for i in range(6):
                for j in range(6):
                    print(self.map[i][j], end=" ")
                print()
            return
        if not self.moving_ctrl("right"):
            self.moving_ctrl("right")
            return
        else:
            self.terminate_turn_cnt += 1
        


    # 복귀하는 함수

    def back(self):
        # self.current_node = 0
        
        self.logger.info("back")
        if not self.stack:
            self.mode = "direction"
            self.terminate_cnt += 1
            if self.terminate_cnt == 2:
                self.mode = "terminate"
            return


        # 스택이 비어있지 않다면 복귀합니다.
        # 가장 이전의 방문했던 노드의 정보를 가져옵니다.
        y, x, dir = self.stack[-1]
        # 현재 방향에서 부모 노드의 방향과 180도 차이가 나지 않는다면
        if abs((self.dir + 2) % 4) != dir:
            # 현재 방향에서 부모의 노드와 180도 차이가 나는 방향으로 회전합니다.
            # small diff angle is left
            # big diff angle is right

            if (self.dir + 2) % 4 > dir:
                if not self.moving_ctrl("right"):
                    self.moving_ctrl("right")
                else:
                    self.dir -= 1
            else:
                if not self.moving_ctrl("left"):
                    self.moving_ctrl("left")
                else:
                    self.dir += 1

        # 회전이 완료되면 전진합니다.
        else:
            if self.x == x and self.y == y:
                self.stack.pop()
            else:
                self.mode = "goback"
                self.stack.pop()
                self.x, self.y = x, y
            return
    def go(self):
        if not self.moving_ctrl("forward"):
            self.moving_ctrl("forward")
        else:
            self.go_flag = False
            return
        

    def control(self):
        # 로봇의 현재 모드를 확인하고 해당 모드에 맞는 함수를 호출합니다.
        # data가 없으면 아무것도 하지 않습니다.
        # self.logger.info("wcrc_ctrl control")
        if self.sensor.odom_msg is not None and self.sensor.camera_msg is not None:
            self.control_dict[self.mode]()
            # self.logger.info(self.mode)
            # self.moving_ctrl("right")
            # print(self.sensor.odom_msg)
