#!/usr/bin/env python3

import rospy
from pymavlink import mavutil
from std_msgs.msg import Bool, Int16

from PyMavlink import ROV

# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

rov = ROV(master)

class Subscriber(object):
    def __init__(self):
        self.isArm = False
        self.pwmDepth = 1500

        rospy.Subscriber('is_arm', Bool, self.callbackIsArm)
        rospy.Subscriber('pwm_depth', Int16, self.callbackPwmDepth)

    def callbackIsArm(self, data):
        self.isArm = data.data

    def callbackPwmDepth(self, data):
        self.pwmDepth = data.data

        if self.isArm:
            rov.setRcValue(3, self.pwmDepth)
            rov.setRcValue(5, 1550)

def main():
    rospy.init_node('node_rc_depth', anonymous=True)

    Subscriber()

    rospy.spin()

if __name__ == '__main__':
    main()