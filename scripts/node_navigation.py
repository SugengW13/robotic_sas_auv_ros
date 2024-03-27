#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from robotic_sas_auv_ros.msg import Sensor, SetPoint, IsStable, Error, ObjectDetection

class Subscriber():
    def __init__(self):
        self.is_object_detected = False

        self.error = Error()
        self.set_point = SetPoint()
        self.is_stable = IsStable()

        # Publisher
        self.pub_error = rospy.Publisher('error', Error, queue_size=10)
        self.pub_is_stable = rospy.Publisher('is_stable', IsStable, queue_size=10)

        # Subscriber
        rospy.Subscriber('sensor', Sensor, self.callback_sensor)
        rospy.Subscriber('object_detection', ObjectDetection, self.callback_object_detection)
        rospy.Subscriber('set_point', SetPoint, self.callback_set_point)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)

    def generate_is_stable(self, thresh, error):
        return -(thresh) <= error <= thresh

    def calculate_orientation_error(self, current, target):
        error = (target - current) % 2
        if error > 1:
            error -= 2
        return error

    def calculate_compass_error(self, current, target):
        return (target - current + 180) % 360 - 180

    # Collect SetPoint Data
    def callback_set_point(self, data: SetPoint):
        self.set_point = data

    # Collect Sensor Data
    def callback_sensor(self, data: Sensor):
        # Calculate Error Value
        error_roll = self.calculate_orientation_error(data.roll, self.set_point.roll)
        error_pitch = self.calculate_orientation_error(data.pitch, self.set_point.pitch)
        error_yaw = self.calculate_orientation_error(data.yaw, self.set_point.yaw)
        error_depth = self.set_point.depth - data.depth

        # Determine Stable Position
        self.is_stable.roll = self.generate_is_stable(0, error_roll)
        self.is_stable.pitch = self.generate_is_stable(0, error_pitch)
        self.is_stable.yaw = self.generate_is_stable(0, error_yaw)
        self.is_stable.depth = self.generate_is_stable(0.05, error_depth)

        # Validate Error Value
        self.error.roll = 0 if self.is_stable.roll else error_roll
        self.error.pitch = 0 if self.is_stable.pitch else error_pitch
        self.error.yaw = 0 if self.is_stable.yaw else error_yaw
        self.error.depth = 0 if self.is_stable.depth else error_depth

    def callback_object_detection(self, data: ObjectDetection):
        self.is_object_detected = len(data.bounding_boxes) > 0

    def callback_is_start(self, data: Bool):
        if data.data:
            self.pub_is_stable.publish(self.is_stable)
            self.pub_error.publish(self.error)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_navigation', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()
