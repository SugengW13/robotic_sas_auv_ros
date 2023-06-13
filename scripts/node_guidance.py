#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Bool, Int16, Float32

class Subscriber():
    def __init__(self):
        # system
        self.boot_time = 0
        self.is_stablilizing = False
        self.start_stable_time = 0
        self.stable_duration = 0

        self.is_object_detected = False
        self.distance_from_center = 0
        self.distance_from_bottom = 0
        self.open_gripper = False
        self.search_object = False
        self.is_object_centered = False
        
        # publisher
        self.pub_search_object = rospy.Publisher('search_object', Bool, queue_size=10)
        self.pub_open_gripper = rospy.Publisher('open_gripper', Bool, queue_size=10)
        self.pub_is_object_centered = rospy.Publisher('is_object_centered', Bool, queue_size=10)

        rospy.Subscriber('/yolo/is_object_detected', Bool, self.callback_is_object_detected)
        rospy.Subscriber('distance_from_center', Int16, self.callback_distance_from_center)
        rospy.Subscriber('distance_from_bottom', Int16, self.callback_distance_from_bottom)
        rospy.Subscriber('boot_time', Float32, self.callback_boot_time)

    def stabilizing_position(self):
        distance = self.distance_from_bottom

        if 100 <= distance <= 150:
            if not self.is_stabilizing:
                self.start_stable_time = self.boot_time

            self.is_stabilizing = True
        else:
            self.stable_duration = 0
            self.is_stabilizing = False

        if self.is_stabilizing:
            self.stable_duration = self.boot_time - self.start_stable_time

        if self.stable_duration >= 3:
            self.open_gripper = True

    def callback_is_object_detected(self, data):
        self.is_object_detected = data.data

        if not self.is_object_detected:
            self.search_object = True
        else:
            self.search_object = False
    
    def callback_distance_from_center(self, data):
        if not self.is_object_detected:
            return

        self.distance_from_center = data.data

        if -50 <= self.distance_from_center <= 50:
            self.is_object_centered = True
        else:
            self.is_object_centered = False

    def callback_distance_from_bottom(self, data):
        if not self.is_object_detected:
            return
        
        self.distance_from_bottom = data.data

        self.stabilizing_position()

    def callback_boot_time(self, data):
        self.boot_time = data.data
        
        self.pub_search_object.publish(self.search_object)
        self.pub_is_object_centered.publish(self.is_object_centered)

        if self.open_gripper:
            self.pub_open_gripper.publish(self.open_gripper)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_guidance', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()