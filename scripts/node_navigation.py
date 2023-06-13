#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int16, Float32

def callback_heading(data):
    print(data.data)

class Subscriber(object):
    def __init__(self):
        # system
        self.boot_time = 0
        self.window_width = 640
        self.window_height = 480

        # yolo
        self.distance_from_center = 0
        self.distance_from_bottom = 0
        self.is_object_detected = False

        # rosserial
        self.heading = 0

        # publisher
        self.pub_distance_from_center = rospy.Publisher('distance_from_center', Int16, queue_size=10)
        self.pub_distance_from_bottom = rospy.Publisher('distance_from_bottom', Int16, queue_size=10)

        # subscriber
        rospy.Subscriber('/yolo/is_object_detected', Bool, self.callback_is_object_detected)
        rospy.Subscriber('/yolo/center_x', Int16, self.callback_center_x)
        rospy.Subscriber('/yolo/center_y', Int16, self.callback_center_y)
        rospy.Subscriber('boot_time', Float32, self.callback_boot_time)

    def callback_is_object_detected(self, data):
        self.is_object_detected = data.data

    def callback_center_x(self, data):
        center_x = data.data
        
        if not self.is_object_detected:
            return

        self.distance_from_center = int(center_x - self.window_width / 2)

    def callback_center_y(self, data):
        center_y = data.data

        if not self.is_object_detected:
            return
        
        self.distance_from_bottom = int(self.window_height - center_y)
    
    def callback_boot_time(self, data):
        self.boot_time = data.data

        if self.is_object_detected:
            self.pub_distance_from_center.publish(self.distance_from_center)
            self.pub_distance_from_bottom.publish(self.distance_from_bottom)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_navigation', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()