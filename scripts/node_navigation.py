#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from robotic_sas_auv_ros.msg import Sensor, SetPoint, Error

class Subscriber():
    def __init__(self):
        self.error = Error()
        self.set_point = SetPoint()

        # Publisher
        self.pub_error = rospy.Publisher('error', Error, queue_size=10)
        self.pub_movement = rospy.Publisher('movement', String, queue_size=10)

        # Subscriber
        rospy.Subscriber('set_point', SetPoint, self.callback_set_point)
        rospy.Subscriber('sensor', Sensor, self.callback_sensor)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)

    def reset_error(self):
        self.error.roll = 0
        self.error.pitch = 0
        self.error.yaw = 0
        self.error.depth = 0

    def calculate_compass_error(self, current, target):
        return (target - current + 180) % 360 - 180

    def callback_set_point(self, data):
        self.set_point = data

    def callback_sensor(self, data):
        error_depth = self.set_point.depth - data.depth

        # is_stable_roll = -3 <= error_roll <= 3
        # is_stable_pitch = -3 <= error_pitch <= 3
        # is_stable_yaw = -3 <= error_yaw <= 3
        is_stable_depth = -0.1 <= error_depth <= 0.1

        # if not is_stable_roll:
        #     rospy.loginfo('STABILIZE ROLL')
        #     self.reset_error()
        #     self.error.roll = error_roll
        #     return

        # if not is_stable_pitch:
        #     rospy.loginfo('STABILIZE PITCH')
        #     self.reset_error()
        #     self.error.pitch = error_pitch
        #     return

        # if not is_stable_yaw:
        #     rospy.loginfo('STABILIZE YAW')
        #     self.reset_error()
        #     self.error.yaw = error_yaw
        #     return

        if not is_stable_depth:
            rospy.loginfo('STABILIZE DEPTH')
            self.reset_error()
            self.error.depth = error_depth
            return

        self.reset_error()

        # Validate stabilize position
        # self.error.roll = 0 if (-1 <= error_roll <= 1) else error_roll
        # self.error.pitch = 0 if (-1 <= error_pitch <= 1) else error_pitch
        # self.error.yaw = 0 if (-1 <= error_yaw <= 1) else error_yaw
        # self.error.depth = 0 if (-0.05 <= error_depth <= 0.05) else error_depth

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