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
from wcrc_ctrl.Sensor import Sensor


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
        self.canny_low, self.canny_high = 100, 120

        # HoughLineP params
        self.hough_threshold, self.min_length, self.min_gap = 10, 50, 10

        # initial state
        self.angle = 0.0
        self.prev_angle = deque([0.0], maxlen=5)
        self.lane = np.array([90.0, 320., 568.])

        # filtering params:
        self.angle_tolerance = np.radians(30)
        self.cluster_threshold = 25

        self.target_lane = 0.0

    def to_canny(self, img, show=False):
        img = cv2.GaussianBlur(img, (7, 7), 0)
        img = cv2.Canny(img, self.canny_low, self.canny_high)
        if show:
            cv2.imshow('canny', img)
            cv2.waitKey(1)
        return img

    def hough(self, img, show=False):
        lines = cv2.HoughLinesP(
            img, 1, np.pi/180, self.hough_threshold, self.min_gap, self.min_length)
        if show:
            hough_img = np.zeros((img.shape[0], img.shape[1], 3))
            if lines is not None:
                for x1, y1, x2, y2 in lines[:, 0]:
                    cv2.line(hough_img, (x1, y1), (x2, y2), red, 2)
            cv2.imshow('hough', hough_img)
            cv2.waitKey(1)
        return lines

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
            self.lane = 1400

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
            return 320
        canny = self.to_canny(img, show=False)
        bev = self.bev(canny, show=False)
        lines = self.hough(bev, show=False)
        positions = self.filter(lines, show=False)
        lane_candidates = self.get_cluster(positions)
        predicted_lane = self.predict_lane()
        self.update_lane(lane_candidates, predicted_lane)
        self.mark_lane(bev)

        return self.target_lane
        # return float(self.target_lane)

