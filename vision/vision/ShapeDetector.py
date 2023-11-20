#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
from .BEV import BEV
from .ORB import ORB

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import numpy as np



class ShapeDetector(object):
    '''
    Detects rectangle shaped contour of given specification range
    '''

    def __init__(self):

        # BEV
        self.bev = BEV()


    def __call__(self, img):
        '''
        return True if stopline is detected else False
        '''
        # cv2.imshow('img', img)
        bev = self.bev(img)
        # 가우시안 블러
        # 5x5 커널을 사용하여 가우시안 블러를 적용합니다.
        # 이유는 노이즈를 제거하기 위해서입니다.
        # blur = cv2.GaussianBlur(bev, (5, 5), 0)

        # # cv2.waitKey(1)
        # # HLS 색공간으로 변환
        # # HLS 색공간은 색상(Hue), 채도(Saturation), 명도(Value)로 구성되어 있습니다.
        # _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        # # 임계값을 적용하여 이진화 이미지를 얻습니다.
        # _, lane = cv2.threshold(
        #     L, self.stopline_threshold, 255, cv2.THRESH_BINARY)
        # # cane = cv2.Canny(L, 50, 150)

        # contours, _ = cv2.findContours(
        #     lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


        # print("rectangle_count : ", rectangle_count)
        cv2.imshow('BEV', bev)
        cv2.waitKey(1)
        
        return bev


class ShapeDetectorNode(Node):

    def __init__(self):
        super().__init__('shape_detection')

        self.bridge = CvBridge()
        self.detector = ShapeDetector()

        self.sub_image = self.create_subscription(
            Image, '/camera2/color/image_raw', self.image_callback, 10)
        
        self.bev_image = self.create_publisher(Image, 'vision/bev_image',10)


    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, y)
            
    def image_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn('Failed to convert image: %s' % e)
            return
        
        cv2.imshow("image",cv_image)

        cv2.setMouseCallback("image", self.mouse_callback)
    
        bev_image = self.bridge.cv2_to_imgmsg(self.detector(cv_image), "bgr8")
        self.bev_image.publish(bev_image)
        



def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
