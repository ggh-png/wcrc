#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError



class ORB:
    def __init__(self):
        target_images = ["/home/ingyu/ros2_ws/src/wcrc/vision/vision/images/star.png"]
        self.target_images = target_images
        self.sift = cv2.ORB_create()

    def compare(self, img, show=False):
        _, query_descriptors = self.sift.detectAndCompute(img, None)
        print(img.dtype)
        print(" ")
        print(query_descriptors)
        print(" ")

        best_match_index = -1
        best_match_score = float('inf')

        for i, target_image_path in enumerate(self.target_images):
            target_image = cv2.imread(target_image_path)
            print(target_image.dtype)
            _, target_descriptors = self.sift.detectAndCompute(target_image, None)

            print(target_descriptors)
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(query_descriptors, target_descriptors)

            match_score = sum([match.distance for match in matches]) / len(matches)

            if match_score < best_match_score:
                best_match_score = match_score
                best_match_index = i

        if show:
            # 여기에 매칭 결과를 시각화하는 코드를 추가할 수 있습니다.
            pass

        return self.target_images[best_match_index]

    def __call__(self, img, show=False):
        result = self.compare(img, show=show)
        return result

