#! /usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class BEV:
    '''
    Calibrates camera images to remove distortion and transforms to bird-eye-view image
    '''

    def __init__(self):
        # calibration config
        self.img_size = (640, 480)
        self.warp_img_w, self.warp_img_h, self.warp_img_mid = 640, 480, 150

        # perspective config

        self.warp_src = np.array([[450,475], [800,475],
                                  [230, 715], [1010,715]], dtype=np.float32)

        # Adjusted to new image width
        self.warp_dist = np.array([[0, 0], [639, 0],
                                   [0, 479], [639,479]], dtype=np.float32)

        self.M = cv2.getPerspectiveTransform(self.warp_src, self.warp_dist)

            
    def to_perspective(self, img, show=False):
        img = cv2.warpPerspective(
            img, self.M, (self.warp_img_w, self.warp_img_h), flags=cv2.INTER_LINEAR)
        if show:
            cv2.imshow('bird-eye-view', img)
            cv2.waitKey(1)
        return img

    def __call__(self, img, show=False):
        '''
        return bird-eye-view image of an input image
        '''
        img = self.to_perspective(img, show=show)
        return img
