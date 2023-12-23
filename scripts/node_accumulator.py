#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16, Float32

class Subscriber():
    def __init__(self):
        # Publisher
        self.pub_roll = rospy.Publisher('roll', Int16, queue_size=10)
        self.pub_pitch = rospy.Publisher('pitch', Int16, queue_size=10)
        self.pub_yaw = rospy.Publisher('yaw', Int16, queue_size=10)
        self.pub_pressure = rospy.Publisher('pressure', Float32, queue_size=10)
        self.pub_temperature = rospy.Publisher('temperature', Float32, queue_size=10)
        self.pub_depth = rospy.Publisher('depth', Float32, queue_size=10)
        self.pub_altitude= rospy.Publisher('altitude', Float32, queue_size=10)

        # Subscriber
        rospy.Subscriber('/arduino/bno_roll', Int16, self.callback_bno_roll)
        rospy.Subscriber('/arduino/bno_pitch', Int16, self.callback_bno_pitch)
        rospy.Subscriber('/arduino/bno_yaw', Int16, self.callback_bno_yaw)
        rospy.Subscriber('/arduino/ms_pressure', Float32, self.callback_ms_pressure)
        rospy.Subscriber('/arduino/ms_temperature', Float32, self.callback_ms_temperature)
        rospy.Subscriber('/arduino/ms_depth', Float32, self.callback_ms_depth)
        rospy.Subscriber('/arduino/ms_altitude', Float32, self.callback_ms_altitude)

    def callback_bno_roll(self, data):
        self.pub_roll.publish(data.data)

    def callback_bno_pitch(self, data):
        self.pub_pitch.publish(data.data)

    def callback_bno_yaw(self, data):
        self.pub_yaw.publish(data.data)

    def callback_ms_pressure(self, data):
        self.pub_depth.publish(data.data)

    def callback_ms_temperature(self, data):
        self.pub_depth.publish(data.data)

    def callback_ms_depth(self, data):
        self.pub_depth.publish(data.data)

    def callback_ms_altitude(self, data):
        self.pub_depth.publish(data.data)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_accumulator', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()