#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from robotic_sas_auv_ros.msg import Sensor, SetPoint, Error

class Subscriber():
    def __init__(self):
        self.error = Error()
        self.set_point = SetPoint()

        self.rate = rospy.Rate(10)

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

    def calculate_orientation_error(self, current, target):
        error = (target - current) % 2
        if error > 1:
            error -= 2
        return error

    def calculate_compass_error(self, current, target):
        return (target - current + 180) % 360 - 180

    def callback_set_point(self, data):
        self.set_point = data

    def callback_sensor(self, data):
        error_roll = self.calculate_orientation_error(data.x, self.set_point.roll)
        error_pitch = self.calculate_orientation_error(data.y, self.set_point.pitch)
        error_yaw = self.calculate_orientation_error(data.z, self.set_point.yaw)
        error_depth = self.set_point.depth - data.depth

        is_stable_roll = -0.1 <= error_roll <= 0.1
        is_stable_pitch = -0.1 <= error_pitch <= 0.1
        is_stable_yaw = -0.1 <= error_yaw <= 0.1
        is_stable_depth = -0.05 <= error_depth <= 0.05

        if not is_stable_roll:
            self.error.roll = error_roll

        if not is_stable_pitch:
            self.error.pitch = error_pitch

        if not is_stable_yaw:
            self.error.yaw = error_yaw

        if not is_stable_depth:
            self.error.depth = error_depth

        # Validate stabilize position
        self.error.roll = 0 if (-0.1 <= error_roll <= 0.1) else error_roll
        self.error.pitch = 0 if (-0.1 <= error_pitch <= 0.1) else error_pitch
        self.error.yaw = 0 if (-0.1 <= error_yaw <= 0.1) else error_yaw
        self.error.depth = 0 if (-0.05 <= error_depth <= 0.05) else error_depth

    def callback_is_start(self, data):
        if data.data:
            self.pub_error.publish(self.error)
        
        self.rate.sleep()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_navigation', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()