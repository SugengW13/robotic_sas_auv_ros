#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from pymavlink import mavutil
from PyMavlink import ROV

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

rov = ROV(master)

def callback(data):
    if data.data == 5:
        rov.arm()

def main():
    rospy.init_node('node_arm', anonymous=True)

    rospy.Subscriber('boot_time', Int16, callback)

    rospy.spin()

if __name__ == '__main__':
    main()