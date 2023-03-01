#!/usr/bin/env python

import cv2
import gi
import numpy as np

from Video import Video

gi.require_version('Gst', '1.0')
from gi.repository import Gst

if __name__ == '__main__':
    video = Video()

    waited = 0
    while not video.frame_available():
        waited += 1
        print(True)
        cv2.waitKey(30)

    while True:
        if video.frame_available():
            frame = video.frame()

            cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break