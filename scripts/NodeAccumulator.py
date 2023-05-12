#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int16, Float32

def callback_is_arm(data):
    print('Is Arm', data.data)

def callback_is_alt_hold(data):
    print('Is Alt Hold', data.data)

def callback_is_unstable_altitude(data):
    print('Is Unstable Alt', data.data)

def callback_is_unstable_heading(data):
    print('Is Unstable Head', data.data)

def callback_boot_time(data):
    print('Boot Time', data.data)

def main():
    rospy.init_node('node_accummulator', anonymous=True)

    rospy.Subscriber('is_arm', Bool, callback_is_arm)
    rospy.Subscriber('is_alt_hold', Bool, callback_is_alt_hold)
    rospy.Subscriber('is_unstable_altitude', Bool, callback_is_unstable_altitude)
    rospy.Subscriber('is_unstable_heading', Bool, callback_is_unstable_heading)
    rospy.Subscriber('boot_time', Int16, callback_boot_time)

    rospy.spin()

if __name__ == '__main__':
    main()