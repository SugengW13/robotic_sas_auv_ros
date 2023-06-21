#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Bool, Int16, Float32, String

class Subscriber():
    def __init__(self):
        # system
        self.is_start = False
        self.boot_time = 0
        self.is_stabilizing = False
        self.start_stable_time = 0
        self.stable_duration = 0

        self.is_object_detected = False
        self.is_stable_altitude = False
        self.is_stable_heading = False
        self.distance_from_center = 0
        self.distance_from_bottom = 0
        self.temp_gripper_command = ''
        self.gripper_command = ''
        self.search_object = False
        self.is_object_centered = False
        
        # publisher
        self.pub_search_object = rospy.Publisher('search_object', Bool, queue_size=10)
        self.pub_is_object_centered = rospy.Publisher('is_object_centered', Bool, queue_size=10)
        self.pub_is_stable_altitude = rospy.Publisher('is_stable_altitude', Bool, queue_size=10)
        self.pub_is_stable_heading = rospy.Publisher('is_stable_heading', Bool, queue_size=10)
        self.pub_gripper_command = rospy.Publisher('gripper_command', String, queue_size=10)

        rospy.Subscriber('/yolo/is_start', Bool, self.callback_is_start)
        rospy.Subscriber('/yolo/is_object_detected', Bool, self.callback_is_object_detected)
        rospy.Subscriber('distance_from_center', Int16, self.callback_distance_from_center)
        rospy.Subscriber('distance_from_bottom', Int16, self.callback_distance_from_bottom)
        rospy.Subscriber('error_altitude', Float32, self.callback_error_altitude)
        rospy.Subscriber('error_heading', Int16, self.callback_error_heading)
        rospy.Subscriber('boot_time', Float32, self.callback_boot_time)

    def stabilizing_position(self):
        if not self.is_start:
            return

        if not self.is_object_centered:
            return

        print(self.stable_duration)

        distance = self.distance_from_bottom
        
        self.temp_gripper_command = self.gripper_command
        
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
            self.gripper_command = 'open'

    def callback_is_start(self, data):
        self.is_start = data.data

    def callback_is_object_detected(self, data):
        if not self.is_start:
            return

        self.is_object_detected = data.data

        if not self.is_object_detected:
            self.search_object = True
        else:
            self.search_object = False
    
    def callback_distance_from_center(self, data):
        if not self.is_start:
            return

        if not self.is_object_detected:
            return

        self.distance_from_center = data.data

        if -50 <= self.distance_from_center <= 50:
            self.is_object_centered = True
        else:
            self.is_object_centered = False

    def callback_distance_from_bottom(self, data):
        if not self.is_start:
            return
        
        if not self.is_object_detected:
            return
        
        self.distance_from_bottom = data.data

        self.stabilizing_position()

    def callback_error_heading(self, data):
        if not self.is_start:
            return
        
        error_heading = data.data

        if -2 < error_heading < 2:
            self.is_stable_heading = True
        else:
            self.is_stable_heading = False

    def callback_error_altitude(self, data):
        if not self.is_start:
            return
        
        error_altitude = data.data

        if -0.05 < error_altitude < 0.05:
            self.is_stable_altitude = True
        else:
            self.is_stable_altitude = False

    def callback_boot_time(self, data):
        self.boot_time = data.data

        if not self.is_start:
            return
        
        self.pub_is_stable_altitude.publish(self.is_stable_altitude)
        self.pub_is_stable_heading.publish(self.is_stable_heading)
        self.pub_search_object.publish(self.search_object)
        self.pub_is_object_centered.publish(self.is_object_centered)

        if self.temp_gripper_command != 'open':
            self.pub_gripper_command.publish(self.gripper_command)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_guidance', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()