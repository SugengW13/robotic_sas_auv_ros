#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

class Subscriber():
    def __init__(self):
        self.target_roll = 0
        self.target_pitch = 0
        self.target_yaw = 0

        self.error_roll = 0
        self.error_pitch = 0
        self.error_yaw = 0

        self.pub_error_roll = rospy.Publisher('error_roll', Int16, queue_size=10)
        self.pub_error_pitch = rospy.Publisher('error_pitch', Int16, queue_size=10)
        self.pub_error_yaw = rospy.Publisher('error_yaw', Int16, queue_size=10)

        rospy.Subscriber('/arduino/roll', Int16, self.callback_roll)
        rospy.Subscriber('/arduino/pitch', Int16, self.callback_pitch)
        rospy.Subscriber('/arduino/yaw', Int16, self.callback_yaw)

    def callback_roll(self, data):
        self.error_roll = data.data - self.target_roll

    def callback_pitch(self, data):
        self.error_pitch = data.data - self.target_pitch

    def callback_yaw(self, data):
        self.error_yaw = data.data - self.target_yaw

    def publish_data(self):
        self.pub_error_roll.publish(self.error_roll)
        self.pub_error_pitch.publish(self.error_pitch)
        self.pub_error_yaw.publish(self.error_yaw)

    def spin(self):
        self.publish_data()
        rospy.spin()

def main():
    rospy.init_node('node_guidance', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()