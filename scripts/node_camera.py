#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    cap = cv2.VideoCapture(0)

    rospy.init_node('node_camera', anonymous=True)

    pub_camera = rospy.Publisher('camera', Image, queue_size=10)

    rate = rospy.Rate(10)

    bridge = CvBridge()

    while not rospy.is_shutdown():
        try:
            ret, frame = cap.read()

            if ret:
                img = bridge.cv2_to_imgmsg(frame, 'bgr8')
                pub_camera.publish(img)
        except KeyboardInterrupt:
            break
            
        rate.sleep()

if __name__ == '__main__':
    main()