#!/usr/bin/env python

import cv2
import time
import rospy
from pymavlink import mavutil
from std_msgs.msg import String

from ObjectDetection import HSV
from PyMavlink import PyMavlink

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

def main(capture, rov: PyMavlink):
    pub = rospy.Publisher('coordinate', String, queue_size=10)

    rospy.init_node('publisher_coordinate', anonymous=True)
    # rospy.init_node('publisher_upper_position', anonymous=True)
    
    rate = rospy.Rate(10)

    # rov.arm()
    # time.sleep(1)
    # rov.setDepth(-0.5)
    # time.sleep(1)
    # rov.setMode('ALT_HOLD')

    while not rospy.is_shutdown():
        _, frame = capture.read()
        
        object_detection = HSV(frame, (2, 90, 40), (25, 255, 255))
        
        upper_position = 0
        coordinate = object_detection.get_center()
        upper_position = object_detection.get_upper_position()
        
        if coordinate is not None and upper_position is not None:
            x, y = int(coordinate[0]), int(coordinate[1])
            str_coordinate = ' '.join(map(str, coordinate))
            
            cv2.line(frame, (0, upper_position), (640, upper_position), (0, 0, 255), 2)
            cv2.line(frame, (x, 0), (x, 480), (0, 255, 0), 2)

            rospy.loginfo(str_coordinate + ' ' + str(upper_position))
            pub.publish(str_coordinate + ' ' + str(upper_position))
        else:
            rospy.loginfo(None)
            pub.publish(None)
        
        cv2.circle(frame, (320, 240), 25, (255, 0, 0), 2)
        cv2.line(frame, (0, 240), (640, 240), (255, 0, 0), 2)

        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    frame.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    pymavlink = PyMavlink(master)

    try:
        main(camera, pymavlink)
    except rospy.ROSInterruptException:
        pass