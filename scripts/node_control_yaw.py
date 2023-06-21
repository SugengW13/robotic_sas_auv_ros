#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float32, Int16, Bool

class Subscriber(object):
    def __init__(self):
        self.boot_time = 0
        self.is_stable_heading = False

        self.kp = 5
        self.kd = 1.5
        self.last_error = 0

        self.pwm_yaw = 0

        self.pub_pwm_yaw = rospy.Publisher('pwm_yaw', Int16, queue_size=10)
        
        rospy.Subscriber('error_heading', Int16, self.callback_error_heading)
        rospy.Subscriber('is_stable_heading', Bool, self.callback_is_stable_heading)
        rospy.Subscriber('boot_time', Float32, self.callback_boot_time)

    def callback_error_heading(self, data):
        error = data.data

        if self.is_stable_heading:
            self.pwm_yaw = 1500
        else:
            term_p = self.kp * error
            term_d = self.kd * (error - self.last_error)

            pid = term_p + term_d

            self.last_error = error

            if pid >= 100:
                pid = 100
            elif pid <= -100:
                pid = -100
            
            self.pwm_yaw = int(1500 + pid)

    def callback_is_stable_heading(self, data):
        self.is_stable_heading = data.data

    def callback_boot_time(self, _):
        self.pub_pwm_yaw.publish(self.pwm_yaw)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_control_depth', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()    

if __name__ == '__main__':
    main()