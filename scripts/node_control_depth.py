#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float32, Int16, Bool

class Subscriber(object):
    def __init__(self):
        self.boot_time = 0
        self.is_stable_depth = False

        self.pwm_throttle = 0

        self.pub_pwm_throttle = rospy.Publisher('pwm_throttle', Int16, queue_size=10)

        rospy.Subscriber('is_stable_depth', Bool, self.callback_is_stable_depth)
        rospy.Subscriber('error_depth', Float32, self.callback_error_depth)
        rospy.Subscriber('boot_time', Float32, self.callback_boot_time)

    def calculate_pwm(self, current_depth):
        if not self.is_stable_depth:
            # Calculate PWM
            self.pwm_throttle = int(np.interp(current_depth, (), ()))
        else:
            self.pwm_throttle = 1500

    def callback_is_stable_depth(self, data):
        self.is_stable_depth = data.data

    def callback_error_depth(self, data):
        self.calculate_pwm(data.data)

    def callback_boot_time(self, _):
        self.pub_pwm_throttle.publish(self.pwm_throttle)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_control_depth', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()    

if __name__ == '__main__':
    main()