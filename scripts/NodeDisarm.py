#!/usr/bin/env python3

import rospy
from pymavlink import mavutil
from std_msgs.msg import Bool, Int16

from PyMavlink import ROV

# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

rov = ROV(master)

def callback(data):
    bootTime = data.data

    if bootTime >= 10:
        print('DISARM')
        rov.disarm()

def main():
    rospy.init_node('node_rc_depth', anonymous=True)

    rospy.Subscriber('boot_time', Int16, callback)

    rospy.spin()

if __name__ == '__main__':
    main()