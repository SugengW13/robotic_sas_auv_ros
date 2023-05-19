import cv2
import numpy as np
from pymavlink import mavutil

from PyMavlink import ROV
from GStreamer import Video
from ObjectDetection import HSV

lower_hsv = np.array([0, 75, 85])
upper_hsv = np.array([35, 255, 255])

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

rov = ROV(master)


def set_rc_forward(pwm):
    rov.setRcValue(5, 1500 + pwm)

def set_rc_lateral(pwm):
    rov.setRcValue(6, 1500 + pwm)

def calculate_pwm_forward(distance):
    pwm = 0
    
    # Min at -200, Max at 0
    # To
    # Min at 100, Max at 50

    if distance < 0:
        if distance < -200:
            pwm = 100
        elif distance > 0:
            pwm = 50
        else:
            pwm = np.interp(distance, (-200, 0), (100, 50))  

    return int(pwm)

def calculate_pwm_lateral(distance):
    # Min at +- 50, Max at +- 350
    # To
    # Min at +- 50, Max at +- 100
    
    pwm = 0

    if distance > 0:
        if distance < 50:
            pwm = 50
        elif distance > 350:
            pwm = 100
        else:
            pwm = np.interp(distance, (50, 350), (50, 100))

    elif distance < 0:
        if distance > -50:
            pwm = -50
        elif distance < -350:
            pwm = -100
        else:
            pwm = np.interp(distance, (-350, -50), (-100, -50))

    return int(pwm)

def guidance(x, y):
    distance_x = x - 400
    distance_y = y - 300

    if 350 < x < 450:
        set_rc_lateral(0)

        if distance_y <= 0:
            print('Go Forward')
            pwm_forward = calculate_pwm_forward(distance_y)
        else:
            print('Drop Object')
            pwm_forward = 0

        set_rc_forward(pwm_forward)
    else:
        set_rc_forward(0)

        if x <= 350:
            print('Go Left')
            pwm_lateral = calculate_pwm_lateral(distance_x)
        elif x >= 450:
            print('Go Right')
            pwm_lateral = calculate_pwm_lateral(distance_x)
        
        set_rc_lateral(pwm_lateral)

def object_detection():
    video = Video()

    while not video.frame_available():
        print('Frame Not Available')

    while True:
        if video.frame_available():
            frame = video.frame()

            bounding_box = HSV(frame, lower_hsv, upper_hsv).get_bounding_box()

            if bounding_box is not None:
                print('Track Object')

                rov.setRcValue(5, 1500)
                
                x, y, w, h = bounding_box

                center_x = int(x + w/2)
                center_y = int(y + h/2)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 1, (0, 255, 0), 2)

                guidance(center_x, center_y)
        
            else:
                print('Search Object')

                rov.setRcValue(5, 1550)

            cv2.line(frame, (350, 0), (350, 600), (255, 0, 0), 1)

            cv2.line(frame, (350, 300), (450, 300), (0, 0, 255), 1)
            cv2.line(frame, (400, 250), (400, 350), (0, 0, 255), 1)

            cv2.line(frame, (450, 0), (450, 600), (255, 0, 0), 1)

            cv2.imshow('Frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

def main():
    rov.arm()

    # Do some movement

    object_detection()

if __name__ == '__main__':
    main()