import cv2
import math
import numpy as np

class HSV():
    def __init__(self, frame, lower_hsv, upper_hsv):
        self.frame = frame
        self.lower_hsv = lower_hsv
        self.upper_hsv = upper_hsv

        self.get_thresh()
    
    def get_contour(self, image):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        return contours

    def get_thresh(self):
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        
        self.thresh = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
    
    def get_center(self):
        contours = self.get_contour(self.thresh)

        if len(contours) > 0:
            centroid = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(centroid)

            if radius > 10:
                return [x, y]
            else:
                return None
        else:  
            return None

    def get_upper_position(self):
        arr = []
        dst = cv2.Canny(self.thresh, 50, 200, None, 3)
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        lines = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 0, 0)
        
        if lines is not None:
            for i in range(0, len(lines)):
                l = lines[i][0]
                arr.append(l[1])
                arr.append(l[3])
                cv2.line(cdst, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
                
            cv2.imshow('thresh', cdst)
            return np.min(arr)
        else:
            return None

class YOLO():
    def __init__(self, frame):
        print(frame)