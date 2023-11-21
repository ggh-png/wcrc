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
        self.img_size = (1280, 960)  # Update to new image size
        self.warp_img_w, self.warp_img_h, self.warp_img_mid = 1280, 960, 320  # Update the output image dimensions accordingly

        # perspective config
        # The source points here would need to be adjusted according to the specific camera setup and the required region of interest.
        # self.warp_src = np.array([[0, 0], [1279, 0],
        #                             [0, 959], [1279, 959]], dtype=np.float32)
        self.warp_src = np.array([[450,475], [800,475],
                                  [230, 715], [1010,715]], dtype=np.float32)
        # Adjusted to new image width and height
        self.warp_dist = np.array([[0, 0], [1279, 0],
                                   [0, 959], [1279, 959]], dtype=np.float32)

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
