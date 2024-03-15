#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    param_cam_index = rospy.get_param('/nuc/cam_index')
    cap = cv2.VideoCapture(param_cam_index)

    rospy.init_node('node_camera', anonymous=True)

    pub_camera = rospy.Publisher('/camera', Image, queue_size=10)

    param_rate = rospy.get_param('/nuc/rate')
    rate = rospy.Rate(param_rate)

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