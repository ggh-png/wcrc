#! /usr/bin/env python3
# -*- coding:utf-8 -*-


import cv2
import sys

image = cv2.imread('images/star.png')
if image is None:
    print('Image load failed')
    sys.exit()
    
cv2.imshow('image', image)
cv2.waitKey(0)