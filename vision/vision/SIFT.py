#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
from vision.BEV import BEV
from vision.MovingAverageFilter import MovingAverageFilter

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

        self.bev = BEV()
        self.avg = MovingAverageFilter()

        
        self.stopline_threshold = 210
        
        self.target_images_7 = [ "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/2.jpg",
                                 "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/8.jpg",
                                 "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/9.jpg",
                                 "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/10.jpg"]
        
        self.shape_7 = ["Cloud", 
                        "Arrow",
                        "Lightning_Bolt",
                        "Moon"]
        
        
        self.target_images_10 =["/home/tetra/ros2_ws/src/wcrc/vision/vision/images/4.jpg",
                                "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/11.jpg"]
        

        self.shape_10 = ["Star",
                        "Heart"]

        self.result = 0
        self.data = []
        self.point_count = 0

    def find_min_value_and_index(self, arr):
        min_value = 10  # 무한대로 초기화
        min_index = -1

        for i in range(len(arr)):
            if arr[i] < min_value:
                min_value = arr[i]
                min_index = i

        return min_value, min_index



    # def mouse_callback(self, event, x, y, flags, param):
    #     if event == cv2.EVENT_LBUTTONDOWN:
    #         print(x, y)
            
            
            
    def matching(self, _final_contours, target_images, shape):
        matchs = []

        for i, target_image_path in enumerate(target_images):
            target_image = cv2.imread(target_image_path)
            target_image = cv2.resize(target_image, (640,640))
            blur_1 = cv2.GaussianBlur(target_image, (5, 5), 0)
            _, L1, _ = cv2.split(cv2.cvtColor(blur_1, cv2.COLOR_BGR2HLS))
            _, lane_1 = cv2.threshold(L1, self.stopline_threshold, 255, cv2.THRESH_BINARY)
            
            _target_contours, _ = cv2.findContours(lane_1, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            match = float(cv2.matchShapes(_final_contours, _target_contours[0], cv2.CONTOURS_MATCH_I3, 0.0))
            matchs.append(match)
                                
        _, min = self.find_min_value_and_index(matchs)                                
        shape_detect = (shape[min])
        print(shape_detect)
        return shape_detect
    
    
    
    def switch(self,_final_contours, point):
        if point == 3: #triangle
            return "Triangle"
        
        elif point == 4: #square
            return "Square"
        
        elif point == 5: #pentagon
            return "Penta"
        
        elif point == 6: #hexagon
            return "Hexagon"
        
        elif point == 7: #cloud, arrow, moon, lightning_bolt
            shape = self.matching(_final_contours, self.target_images_7, self.shape_7)
            return shape
        
        elif point == 8: # circle
            return "Circle"
        
        elif point == 10: # heart, star
            shape = self.matching(_final_contours, self.target_images_10, self.shape_10)
            return shape

        elif point == 12: # cross
            return "Cross"  
        
        else : 
            return "None"      
                
            
            
    def __call__(self, img):
        
        
        x = 90
        y = 50
        w = 290
        h = 200
        
        cv2.imshow("img", img)
            #일어온거 컨투어
        bev = self.bev(img)
        blur = cv2.GaussianBlur(bev, (5, 5), 0)
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        equal = cv2.equalizeHist(L)
        _, lane = cv2.threshold(equal, self.stopline_threshold, 255, cv2.THRESH_BINARY)
        cv2.imshow("line", lane)
        roi = lane[y:y+h, x:x+w]
        roi1 = bev[y:y+h, x:x+w]
        
                
        _vision_contours, _ = cv2.findContours(roi, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        count = len(_vision_contours)
        
        for i in range(count):
            area = int(cv2.contourArea(_vision_contours[i]))
            print(area)
            
            if area > 1000 and area < 4000:
                _final_contours = _vision_contours[i]
                epsilon = 0.02 * cv2.arcLength(_final_contours, True)
                approx = cv2.approxPolyDP(_final_contours, epsilon, True)
                
                                
                for point in approx:
                    cv2.circle(roi1, tuple(point[0]), 5, (0, 255, 0), -1)
            
            
            
            cv2.drawContours(roi1, _vision_contours[i], -1, (0, 255, 0), 2)
            break
                    
            # point_count = (len(approx))
            # final_shape = self.switch(_final_contours, point_count)
            # print(final_shape)
            
            # print(point_count)

             

        
        cv2.imshow("bev",bev)
        cv2.imshow("roi1",roi1)
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


    def image_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn('Failed to convert image: %s' % e)
            return
        
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
