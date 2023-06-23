#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Int16, Bool
from pymavlink import mavutil
from PyMavlink import ROV

class Subscriber(object):
    def __init__(self, rov: ROV):
        self.rov = rov

        self.is_start = False
        self.pwm_throttle = 1500
        self.pwm_forward= 1500
        self.pwm_lateral = 1500

        # subscriber
        rospy.Subscriber('/yolo/is_start', Bool, self.callback_is_start)
        rospy.Subscriber('pwm_throttle', Int16, self.callback_pwm_throttle)
        rospy.Subscriber('pwm_yaw', Int16, self.callback_pwm_yaw)
        rospy.Subscriber('pwm_lateral', Int16, self.callback_pwm_lateral)
        rospy.Subscriber('pwm_forward', Int16, self.callback_pwm_forward)

    def callback_is_start(self, data):
        print(data)
        self.is_start = data.data

        if self.is_start:
            self.rov.arm()

    def callback_pwm_throttle(self, data):
        if not self.is_start:
            return
        
        pwm_throttle = data.data
        
        self.rov.setRcValue(3, pwm_throttle)

    def callback_pwm_yaw(self, data):
        if not self.is_start:
            return
        
        pwm_yaw = data.data

        self.rov.setRcValue(4, pwm_yaw)

    def callback_pwm_forward(self, data):
        if not self.is_start:
            return

        pwm_forward = data.data
        self.rov.setRcValue(5, pwm_forward)

    def callback_pwm_lateral(self, data):
        if not self.is_start:
            return

        pwm_lateral = data.data
        self.rov.setRcValue(6, pwm_lateral)

    def spin(self):
        rospy.spin()

def main():
    pixhawk_connection = rospy.get_param('/auv/pixhawk_connection')
    pixhawk_address = rospy.get_param('/auv/pixhawk_address')

    if pixhawk_connection == 'usb':
        master = mavutil.mavlink_connection(pixhawk_address, baud=115200)
    else:
        master = mavutil.mavlink_connection(pixhawk_address)

    rov = ROV(master)

    rospy.init_node('node_control_auv', anonymous=True)

    subscriber = Subscriber(rov)

    subscriber.spin()

if __name__ == '__main__':
    main()