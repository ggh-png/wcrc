import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
from collections import deque
from wcrc_ctrl.BEV import BEV
from wcrc_ctrl.Logger import Logger
# from wcrc_ctrl.Sensor import Sensor


# colors
red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)


class LaneDetector:
    '''
    Detects left, middle, right lane from an image and calculate angle of the lane.
    Uses canny, houghlinesP for detecting possible lane candidates.
    Calculates best fitted lane position and predicted lane position from previous result.
    '''

    def __init__(self, node: Node = None):
        # Ensure that the node argument is indeed an instance of rclpy.node.Node
        if not isinstance(node, Node):
            raise TypeError("Logger expects an rclpy.node.Node instance.")

        self.bev = BEV()

        # canny params
        self.canny_low, self.canny_high = 40, 120

        # HoughLineP params
        self.hough_threshold, self.min_length, self.max_gap = 5, 20, 10

        # initial state
        self.angle = 0.0
        self.prev_angle = deque([0.0], maxlen=5)
        self.lane = np.array([1280])

        # filtering params:
        self.angle_tolerance = np.radians(30)
        self.cluster_threshold = 25

        self.target_lane = 0.0

    def to_canny(self, img, show=False):
        img = cv2.GaussianBlur(img, (9, 9), 0)
        img = cv2.Canny(img, self.canny_low, self.canny_high)
        if show:
            cv2.imshow('canny', img)
            cv2.waitKey(1)
        return img

    def hough(self, img, show=False):

        lines = cv2.HoughLinesP(
            img, 1, np.pi / 180, self.hough_threshold, self.min_length, self.max_gap)
        if show:
            hough_img = np.zeros((img.shape[0], img.shape[1], 3))
            if lines is not None:
                for x1, y1, x2, y2 in lines[:, 0]:
                    cv2.line(hough_img, (x1, y1), (x2, y2), red, 2)
            cv2.imshow('hough', hough_img)
            cv2.waitKey(1)
        return lines

    def find_best_pair(self, lines, img):
        pass

    def draw_lines(self, img, lines):
        width = img.shape[1]

        if lines is not None:
            for x1, y1, x2, y2 in lines:
                if x2 - x1 == 0:
                    intercept = y1
                    start_point = (0, int(intercept))  # Start at x = 0
                    end_point = (
                        width - 1, int(intercept))
                    cv2.line(img, start_point,
                             end_point, (255, 0, 0), 3)
                else:
                    slope = (y2 - y1) / (x2 - x1)
                    intercept = y1 - slope * x1

                    start_point = (0, int(intercept))  # Start at x = 0
                    end_point = (
                        width - 1, int(slope * (width - 1) + intercept))
                    cv2.line(img, start_point,
                             end_point, (255, 0, 0), 3)

    def filter(self, lines, show=True):
        '''
        filter lines that are close to previous angle and calculate its positions
        '''

        thetas, positions = [], []  # ??? ??? ???? ?? ??? ???
        if show:
            # ??? ??? ??? ? ??? ???
            filter_img = np.zeros(
                (self.bev.warp_img_h, self.bev.warp_img_w, 3))

        if lines is not None:  # ??? ???? ??? ??
            for x1, y1, x2, y2 in lines[:, 0]:  # ? ??? ??
                if y1 == y2:  # ??? ??? ?? ??
                    continue

                # ??? ??? ?? ??? ??
                flag = 1 if y1-y2 > 0 else -1

                # ??? ?? ??
                theta = np.arctan2(flag * (x2-x1), flag * 0.9 * (y1-y2))

                # ??? ??? ??? ?? ??? ??? ??? ??
                if abs(theta - self.angle) < self.angle_tolerance:
                    # ?? ??? ?? ??
                    position = float(
                        (x2-x1)*(self.bev.warp_img_mid-y1))/(y2-y1) + x1
                    thetas.append(theta)  # ??? ?? ??
                    positions.append(position)  # ??? ?? ??
                    if show:  # ???? ?? ??
                        cv2.line(filter_img, (x1, y1), (x2, y2), red, 2)

        # ?? ?? ???? ?? ?? ??
        self.prev_angle.append(self.angle)

        # ??? ???? ????? ?? ?? ????
        if thetas:
            self.angle = np.mean(thetas)

        if show:  # ???? ???? ??
            cv2.imshow('filtered lines', filter_img)
            cv2.waitKey(1)
        return positions  # ??? ??? ??

    def get_cluster(self, positions):
        '''
        group positions that are close to each other
        '''

        clusters = []  # ????? ???? ?? ??? ???

        # ? ??? ??
        for position in positions:
            # ?? ??? ??? ?? ?? ?? ??
            if 0 <= position < self.bev.warp_img_w:
                for cluster in clusters:
                    # ?? ??? ?? ??? ????? ?? ?? ??
                    if abs(cluster[0] - position) < self.cluster_threshold:
                        # ????? ?? ??
                        cluster.append(position)
                        break
                else:
                    # ?? ?????? ??? ?? ?? ??? ???? ??
                    clusters.append([position])

        # ? ????? ?? ?? ??
        lane_candidates = [np.mean(cluster) for cluster in clusters]
        # print(lane_candidates)
        return lane_candidates  # ??? ?? ??? ??

    def predict_lane(self):
        '''
        predicts the center lane position from the previous center lane position and angle
        '''
        # self.lane[1]? ?? ?? ???? ??? ?:
        if isinstance(self.lane, (list, np.ndarray)) and len(self.lane) > 1:
            # ?? ?? ????? ??? 2?? ??? ?? ?? ?? ?? ?? ??
            center_lane = self.lane[1]
        else:
            # self.lane? ??? ?? ?? ?? ??
            center_lane = self.lane

        # ??? ?? ?? ?? ??
        predicted_center_lane = center_lane + \
            (self.angle - np.mean(self.prev_angle)) * 70
        # print(predicted_center_lane)
        return predicted_center_lane

    def update_lane(self, lane_candidates, predicted_lane):
        # If predicted_lane is an array with one element, convert it to a scalar
        if isinstance(predicted_lane, np.ndarray) and predicted_lane.size == 1:
            predicted_lane = predicted_lane.item()

        # Find the lane candidate closest to the predicted lane
        if lane_candidates:
            # ??? ??? ??? ??? ???? ?? ??? ?? ????.
            closest_lane = min(
                lane_candidates, key=lambda lc: abs(lc - predicted_lane))
            # ???? ???????.
            self.lane = closest_lane
        else:
            # ?? ??? ??? ??? ???? ?????.
            self.lane = 1004

    def mark_lane(self, img, lane=None):
        '''
        mark calculated lane position to an image 
        '''
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if lane is None:
            lane = self.lane
        l1 = self.lane
        self.target_lane = l1
        cv2.circle(img, (int(l1), self.bev.warp_img_mid),
                   3, red, 5, cv2.FILLED)
        # print(l1)
        # cv2.circle(img, (int(l2), self.bev.warp_img_mid),
        #            3, green, 5, cv2.FILLED)
        # cv2.circle(img, (int(l3), self.bev.warp_img_mid),
        #            3, blue, 5, cv2.FILLED)
        cv2.imshow('marked', img)
        cv2.waitKey(1)

    def __call__(self, img):
        '''
        returns angle and cte of a target lane from an image
        angle : radians
        cte : pixels
        '''
        if img is None:
            return 1004

        canny = self.to_canny(img, show=True)
        image_height = img.shape[0]
        image_width = img.shape[1]
        x_padding = 500
        y_top_padding = 660
        # 1280 x 720
        roi_org = img[y_top_padding:720, x_padding:image_width-x_padding]
        bev = canny[y_top_padding:720, x_padding:image_width-x_padding]
        window_width = 60
        window_height = bev.shape[0]

        window_step = 20
        i = 0
        mean = [0, 0, 0]
        means = []
        black_min_means = [150, 150, 150]
        yellow_min_means = [120, 200, 230]
        yellow_max_means = [160, 255, 255]

        goal_means = yellow_min_means
        goal_max_means = yellow_max_means
        detected = False
        while True:
            start_x = window_step * i
            end_x = start_x + window_width

            if end_x > bev.shape[1]:
                break

            roi = roi_org[0:window_height, start_x:end_x]
            mean_values = cv2.mean(roi)

            means.append(mean_values)
            mean[0] += mean_values[0]
            mean[1] += mean_values[1]
            mean[2] += mean_values[2]

            r_valid = goal_means[0] <= mean_values[0] <= goal_max_means[0]
            g_valid = goal_means[1] <= mean_values[1] <= goal_max_means[1]
            b_valid = goal_means[2] <= mean_values[2] <= goal_max_means[2]

            if r_valid and g_valid and b_valid:
                detected = True
                
            # print(mean_values)
            # cv2.imshow('max_roi', roi)
            # cv2.waitKey(0)

            i += 1

        print("detected: ", detected)
        # bev = self.bev(canny, show=True)
        lines = self.hough(bev, show=True)

        cv2.imshow('bev', bev)
        cv2.waitKey(1)

        positions = self.filter(lines, show=False)
        lane_candidates = self.get_cluster(positions)
        predicted_lane = self.predict_lane()
        self.update_lane(lane_candidates, predicted_lane)
        self.mark_lane(bev)

        return self.target_lane
        # return float(self.target_lane)
