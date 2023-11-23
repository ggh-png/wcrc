#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
from vision.BEV import BEV
from vision.MovingAverageFilter import MovingAverageFilter

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np



class ShapeDetector(object):
    '''
    Detects rectangle shaped contour of given specification range
    '''

    def __init__(self):

        self.bev = BEV()
        
        self.stopline_threshold = 180


        self.target_images_6 = ["/home/tetra/ros2_ws/src/wcrc/vision/vision/images/6.jpg",
                                 "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/10.jpg"]
                
        self.shape_6 = ["Hexagon",
                        "Moon"]
        self.target_images_7 = [ "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/2.jpg",
                                 "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/8.jpg",
                                 "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/14.jpg",
                                 "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/9.jpg",
                                 "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/10.jpg"]
        
        self.shape_7 = ["Cloud", 
                        "Arrow",
                        "Arrow",
                        "Lightning_Bolt",
                        "Moon"]
        

        self.target_images_8 = [ "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/5.jpg",
                                 "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/2.jpg"]
        
        self.shape_8 = ["Circle",
                       "Cloud"]
        
        
        self.target_images_10 =["/home/tetra/ros2_ws/src/wcrc/vision/vision/images/4.jpg",
                                "/home/tetra/ros2_ws/src/wcrc/vision/vision/images/11.jpg"]
        

        self.shape_10 = ["Star",
                        "Heart"]

        self.result = 0
        self.data = []
        self.point_count = 0



    def find_min_value_and_index(self, arr):
        min_value = 100  # ???? ???
        min_index = -1

        for i in range(len(arr)):
            print(arr[i])
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
            img1 = target_images.copy()
            target_image = cv2.resize(target_image, (640,640))
            blur_1 = cv2.GaussianBlur(target_image, (5, 5), 0)
            _, L1, _ = cv2.split(cv2.cvtColor(blur_1, cv2.COLOR_BGR2HLS))
            _, lane_1 = cv2.threshold(L1, self.stopline_threshold, 255, cv2.THRESH_BINARY)
            
            _target_contours, _ = cv2.findContours(lane_1, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

            match = float(cv2.matchShapes(_final_contours, _target_contours[0], cv2.CONTOURS_MATCH_I2, 0.0))
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
            shape = self.matching(_final_contours, self.target_images_6, self.shape_6)
            return shape
        
        elif point == 7: #cloud, arrow, moon, lightning_bolt
            shape = self.matching(_final_contours, self.target_images_7, self.shape_7)
            return shape
        
        elif point == 8: # circle
            shape = self.matching(_final_contours, self.target_images_8, self.shape_8)
            return shape
        
        elif point == 9:
            return "Heart"
        
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
            #???? ???
        bev = self.bev(img)
        blur = cv2.GaussianBlur(bev, (5, 5), 0)
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        equal = cv2.equalizeHist(L)
        _, lane = cv2.threshold(equal, self.stopline_threshold, 255, cv2.THRESH_BINARY)
        cv2.imshow("line", lane)
        roi = lane[y:y+h, x:x+w]
        roi1 = bev[y:y+h, x:x+w]
        
        # ? ???
        _vision_contours, _ = cv2.findContours(lane, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        count = len(_vision_contours)
        cv2.drawContours(roi1, _vision_contours, -1, (0, 255, 0), 2)
        
        _contour = sorted(_vision_contours, key=cv2.contourArea, reverse=True)
        
        try:
            final_shape = _contour[1]
            epsilon = 0.019 * cv2.arcLength(final_shape, True)
            approx = cv2.approxPolyDP(final_shape, epsilon, True)
                
                                
            for point in approx:
                cv2.circle(roi1, tuple(point[0]), 5, (0, 255, 0), -1)

            point_count = (len(approx))
            final_shape = self.switch(final_shape, point_count)
             

        except:
            final_shape = "None"
    

        # print(final_shape)`
                
        # print(point_count)

        # return final_shape             

        
        # cv2.imshow("bev",bev)
        # cv2.imshow("roi1",roi1)
        # print(final_shape)
        # cv2.waitKey(1)
        
        
        return final_shape
        

        
        



class ShapeDetectorNode(Node):

    def __init__(self):
        super().__init__('shape_detection')

        self.bridge = CvBridge()
        self.detector = ShapeDetector()

        self.sub_image = self.create_subscription(
            Image, '/camera2/color/image_raw', self.image_callback, 10)
        
        self.pub_shape = self.create_publisher(String, 'shape',10)
        self.shape_msg = String()
        
        self.bev_image = self.create_publisher(Image, 'vision/bev_image',10)
        

    def image_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn('Failed to convert image: %s' % e)
            return
        
        self.shape_msg.data = self.detector(cv_image)
        self.pub_shape.publish(self.shape_msg)
        
        # bev_image = self.bridge.cv2_to_imgmsg(self.detector(cv_image), "bgr8")
        # self.bev_image.publish(bev_image)
        



def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
