#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class WebcamNode:
    def __init__(self):
        self.bridge = CvBridge()

        # Publisher for image
        self.pub_img = rospy.Publisher('webcam_image', Image, queue_size=10)

        # Open the webcam
        self.cap = cv2.VideoCapture(0)

        # Set the resolution to 480p
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Define the codec and create a VideoWriter object
        ret, frame = self.cap.read()
        if ret:
            height, width, _ = frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'XVID')

            # Get the current time to use in the filename
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f'output_{timestamp}.avi'
            
            self.out = cv2.VideoWriter(filename, fourcc, 15.0, (width, height))  # Set fps to 15

                        
    def process_frame(self):
        # Read a frame from the webcam
        ret, frame = self.cap.read()

        if not ret:
            rospy.logerr("Unable to capture video")
            return

        # Write the frame into the file 'output.avi'
        self.out.write(frame)

        # Convert the image back to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        # Publish the image
        self.pub_img.publish(img_msg)

    def close(self):
        # Release everything if job is finished
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('webcam_node')
    node = WebcamNode()

    rate = rospy.Rate(10) # 10 Hz
    try:
        while not rospy.is_shutdown():
            node.process_frame()
            rate.sleep()
    finally:
        node.close()