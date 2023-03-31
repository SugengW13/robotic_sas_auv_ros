#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import cv2

from ObjectDetection import HSV

def main(capture):
    lowerHsv = (156, 30, 112)
    upperHsv = (188, 255, 255)

    pubY = rospy.Publisher('y_coordinate', Float32, queue_size=10)
    pubX = rospy.Publisher('x_coordinate', Float32, queue_size=10)
    pubUpper = rospy.Publisher('upper_coordinate', Float32, queue_size=10)

    rospy.init_node('node_color_detection', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ret, frame = capture.read()

        hsv = HSV(frame, lowerHsv, upperHsv)

        coordinate = hsv.get_center()
        upperCoordinate = hsv.get_upper_position()

        print(upperCoordinate)

        if coordinate != None and upperCoordinate != None:
            x = coordinate[0]
            y = coordinate[1]

            cv2.line(frame, (int(x), 0), (int(x), 480), (0, 255, 0), 2)
            cv2.line(frame, (0, upperCoordinate), (640, upperCoordinate), (0, 255, 0), 2)
            
            pubX.publish(x)
            pubY.publish(y)
            pubUpper.publish(upperCoordinate)
        else:
            print('Search Object')

        cv2.imshow('Frame', frame)

        rate.sleep()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    camera = cv2.VideoCapture(0)
    camera.set(3, 640)
    camera.set(4, 480)

    try:
        main(camera)
    except rospy.ROSInterruptException:
        pass