#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from std_msgs.msg import Int16MultiArray

from ObjectDetection import HSV

cap = cv2.VideoCapture(0)

lower_hsv = np.array([0, 75, 85])
upper_hsv = np.array([35, 255, 255])

def main():
    while True:
        _, frame = cap.read()

        # object_detection = HSV(frame, lower_hsv, upper_hsv)

        # bounding_box = object_detection.get_bounding_box()

        # if bounding_box is not None:
        #     print(bounding_box)

        cv2.imshow('Frame', frame)

if __name__ == '__main__':
    main()