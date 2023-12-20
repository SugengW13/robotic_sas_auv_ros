#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16, Float32

class Subscriber():
    def __init__(self):
        # Publisher
        self.pub_roll = rospy.Publisher('roll', Int16, queue_size=10)
        self.pub_pitch = rospy.Publisher('pitch', Int16, queue_size=10)
        self.pub_yaw = rospy.Publisher('yaw', Int16, queue_size=10)
        self.pub_depth = rospy.Publisher('depth', Float32, queue_size=10)

        # Subscriber
        rospy.Subscriber('/arduino/bno_roll', Int16, self.callback_bno_roll)
        rospy.Subscriber('/arduino/bno_pitch', Int16, self.callback_bno_pitch)
        rospy.Subscriber('/arduino/bno_yaw', Int16, self.callback_bno_yaw)
        rospy.Subscriber('/arduino/br_depth', Float32, self.callback_br_depth)

    def callback_bno_roll(self, data):
        self.pub_roll.publish(data.data)

    def callback_bno_pitch(self, data):
        self.pub_pitch.publish(data.data)

    def callback_bno_yaw(self, data):
        self.pub_yaw.publish(data.data)

    def callback_br_depth(self, data):
        self.pub_depth.publish(data.data)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_guidance', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()