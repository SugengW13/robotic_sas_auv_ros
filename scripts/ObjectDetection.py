import cv2

class HSV():
    def __init__(self, frame, lower_hsv, upper_hsv):
        self.frame = frame
        self.lower_hsv = lower_hsv
        self.upper_hsv = upper_hsv
        
    def get_bounding_box(self):
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            if w >= 50 or h >= 50:
                return x, y, w, h
            else:
                return None
        else:
            return None

class YOLO():
    def __init__(self, frame):
        print(frame)