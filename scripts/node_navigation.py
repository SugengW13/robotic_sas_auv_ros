#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int16, Float32

class Subscriber():
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.depth = 0.0
        self.error_roll = 0
        self.error_pitch = 0
        self.error_yaw = 0
        self.error_depth = 0.0

        # Publisher
        self.pub_error_roll = rospy.Publisher('error_roll', Int16, queue_size=10)
        self.pub_error_pitch = rospy.Publisher('error_pitch', Int16, queue_size=10)
        self.pub_error_yaw = rospy.Publisher('error_yaw', Int16, queue_size=10)
        self.pub_error_depth = rospy.Publisher('error_depth', Float32, queue_size=10)

        # Subscriber
        rospy.Subscriber('roll', Int16, self.callback_roll)
        rospy.Subscriber('pitch', Int16, self.callback_pitch)
        rospy.Subscriber('yaw', Int16, self.callback_yaw)
        rospy.Subscriber('depth', Float32, self.callback_depth)
        rospy.Subscriber('target_roll', Int16, self.callback_target_roll)
        rospy.Subscriber('target_pitch', Int16, self.callback_target_pitch)
        rospy.Subscriber('target_yaw', Int16, self.callback_target_yaw)
        rospy.Subscriber('target_depth', Float32, self.callback_target_depth)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)

    def callback_roll(self, data):
        self.roll = data.data

    def callback_pitch(self, data):
        self.pitch = data.data

    def callback_yaw(self, data):
        self.yaw = data.data
    
    def callback_depth(self, data):
        self.depth = data.data

    def callback_target_roll(self, data):
        self.error_roll = data.data - self.roll

    def callback_target_pitch(self, data):
        self.error_pitch = data.data - self.pitch

    def callback_target_yaw(self, data):
        self.error_yaw = data.data - self.yaw

    def callback_target_depth(self, data):
        self.error_depth = data.data - self.depth

    def callback_is_start(self, _):
        self.pub_error_roll.publish(self.error_roll)
        self.pub_error_pitch.publish(self.error_pitch)
        self.pub_error_yaw.publish(self.error_yaw)
        self.pub_error_depth.publish(self.error_depth)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_navigation', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()