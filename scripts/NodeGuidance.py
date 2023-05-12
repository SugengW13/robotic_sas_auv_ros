#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int16, Float32

is_arm = False
is_alt_hold = False
altitude = 0.0
heading = 0

pub_is_arm = rospy.Publisher('is_arm', Bool, queue_size=10)
pub_is_alt_hold = rospy.Publisher('is_alt_hold', Bool, queue_size=10)

def callback_base_mode(data):
    global is_arm

    if data.data == 209:
        is_arm = True

def callback_custom_mode(data):
    global is_alt_hold
    
    if data.data == 2:
        is_alt_hold = True

def callback_altitude(data):
    global altitude
    altitude = data.data

def callback_heading(data):
    global heading
    heading = data.data

def callback_boot_time(_):
    global is_arm, is_alt_hold, altitude, heading

    # print(is_arm, is_alt_hold, altitude, heading)

    pub_is_arm.publish(is_arm)
    pub_is_alt_hold.publish(is_alt_hold)

def main():
    rospy.init_node('node_guidance', anonymous=True)

    rospy.Subscriber('base_mode', Int16, callback_base_mode)
    rospy.Subscriber('custom_mode', Int16, callback_custom_mode)
    rospy.Subscriber('altitude', Float32, callback_altitude)
    rospy.Subscriber('heading', Int16, callback_heading)
    rospy.Subscriber('boot_time', Int16, callback_boot_time)

    rospy.spin()

if __name__ == '__main__':
    main()