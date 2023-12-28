#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from robotic_sas_auv_ros.msg import Sensor, SetPoint, Error

class Subscriber():
    def __init__(self):
        self.error = Error()
        self.set_point = SetPoint()

        # Publisher
        self.pub_error = rospy.Publisher('error', Error, queue_size=10)

        # Subscriber
        rospy.Subscriber('set_point', SetPoint, self.callback_set_point)
        rospy.Subscriber('sensor', Sensor, self.callback_sensor)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)

    def callback_set_point(self, data):
        self.set_point = data

    def callback_sensor(self, data):
        error_roll = data.roll - self.set_point.roll
        error_pitch = data.pitch - self.set_point.pitch
        error_yaw = data.yaw - self.set_point.yaw
        error_depth = self.set_point.depth - data.depth

        # Validate stabilize position
        self.error.roll = 0 if (-1 <= error_roll <= 1) else error_roll
        self.error.pitch = 0 if (-1 <= error_pitch <= 1) else error_pitch
        self.error.yaw = 0 if (-1 <= error_yaw <= 1) else error_yaw
        self.error.depth = 0 if (-0.05 <= error_depth <= 0.05) else error_depth

    def callback_is_start(self, data):
        if data.data:
            self.pub_error.publish(self.error)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_navigation', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()