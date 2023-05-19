#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Int16, Float32
from pymavlink import mavutil
from PyMavlink import ROV

import os
os.environ['MAVLINK20'] = ''

# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

rov = ROV(master)

def callback_boot_time(data):
    print(data.data)

def main():
    rospy.init_node('node_control_rov', anonymous=True)

    rospy.Subscriber('node_boot_time', Int16, callback_boot_time)

    rospy.spin()

if __name__ == '__main__':
    rov.arm()

    main()