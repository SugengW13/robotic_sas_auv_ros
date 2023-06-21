#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float32, Int16, Bool

class Subscriber(object):
    def __init__(self):
        self.boot_time = 0
        self.is_stable_altitude = False

        self.pwm_throttle = 0

        self.pub_pwm_throttle = rospy.Publisher('pwm_throttle', Int16, queue_size=10)
        
        rospy.Subscriber('error_altitude', Float32, self.callback_error_altitude)
        rospy.Subscriber('is_stable_altitude', Bool, self.callback_is_stable_altitude)
        rospy.Subscriber('boot_time', Float32, self.callback_boot_time)

    def callback_error_altitude(self, data):
        error = data.data

        if self.is_stable_altitude:
            self.pwm_throttle = 1500
        else:
            if error >= 0.05:
                self.pwm_throttle = int(np.interp(error, (0.05, 0.5), (1525, 1600)))
            elif error <= -0.05:
                self.pwm_throttle = int(np.interp(error, (-0.5, -0.05), (1400, 1475)))

        if self.pwm_throttle >= 1600:
            self.pwm_throttle = 1600
        elif self.pwm_throttle <= 1400:
            self.pwm_throttle = 1400
        
    def callback_is_stable_altitude(self, data):
        self.is_stable_altitude = data.data

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