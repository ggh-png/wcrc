#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String

from cv_bridge import CvBridge, CvBridgeError

import numpy as np


# colors
class TrafficLightDetector(object):
    '''
    Detects rectangle shaped contour of given specification range
    '''

    def __init__(self):
        self.traffic_light = 'Red'
        self.detected = False
        self.traffic_light_threshold = 100

    def detect_traffic_light(self, image):
        # 640x480? ???? 320? 480?? ????
        # image = image[:250, 200:440]
        cv2.imshow('image', image)
        # ?? ???? ???? ??? ???? ???? ?????.
        # 248 164, 382 206
        # 5x5 ??? ???? ???? ??? ?????.
        # ???? ??? ???? ???? ?????.
        blur = cv2.GaussianBlur(image, (5, 5), 0)
        # cv2.imshow('blur', blur)
        # HLS ????? ???? ??? ?????.
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        # cv2.imshow('L', L)
        # ???? ???? ??? ???? ????.

        _, lane = cv2.threshold(
            L, self.traffic_light_threshold, 255, cv2.THRESH_BINARY)
        cv2.imshow('lane', lane)
        # Canny ?? ??
        edges = cv2.Canny(lane, 70, 200)
        cv2.imshow('edges', edges)

        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # ???? ???? ?? ?? ????? ????.
        self.detected = False
        for cont in contours:
            # ???? ???? ?? ?? ????? ????.
            # ??? ???? ??? ??, ???? ??? ??? 4?? ?? ? ????.
            # ??? ??? ??? 3? ??? ???? ????? ?????.
            if len(cont) != 3:
                continue
            x, y, w, h = cv2.boundingRect(cont)

            # ??? ???? ??? 30, ??? 60? ??? ?????? ?????.
            # ?? ??? +/- 10?? ???????.
            # 23, 57 640?480
            # 33, 79

            if (40 <= w <= 60) and (150 <= h):
                self.detected = True
                # ?? ??? ???? ???? ??? ??? ???? ????.
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # cnt += 1

        print(self.detected)
        return self.detected
        # print(self.detected)
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    def __call__(self, image):
        self.traffic_light = self.detect_traffic_light(image)
        cv2.imshow('Traffic Light Detection', image)
        cv2.waitKey(1)
        return self.traffic_light


class TrafficLightDetectionNode(Node):

    def __init__(self):
        super().__init__('stop_line_detection_node')

        self.bridge = CvBridge()
        self.detector = TrafficLightDetector()

        self.sub_image = self.create_subscription(
            Image, '/camera2/color/image_raw', self.image_callback, 10)

        # You may want to publish the detected lane or some other information
        # For simplicity, we'll republish the image with the lane markings
        self.pub_centor_lane = self.create_publisher(
            String, '/traffic_light', 10)

    def image_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn('Failed to convert image: %s' % e)
            return

        # Use the LaneDetector logic here
        detected = self.detector(cv_image)  # As an example
        print(detected)

        # Publish the detected lane
        # self.pub_centor_lane.publish(String(data=detected))


def main(args=None):
    rclpy.init(args=args)

    node = TrafficLightDetectionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
