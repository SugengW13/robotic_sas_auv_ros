#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int16

adjust_altitude = False
adjust_heading = False

pub_adjust_altitude = rospy.Publisher('adjust_altitude', Bool, queue_size=10)
pub_adjust_heading = rospy.Publisher('adjust_heading', Bool, queue_size=10)

def callback_is_arm(data):
    print('Is Arm', data.data)

def callback_is_alt_hold(data):
    print('Is Alt Hold', data.data)

def callback_is_unstable_altitude(data):
    global adjust_altitude
    
    if data.data:
        adjust_altitude = False
    else:
        adjust_altitude = True

def callback_is_unstable_heading(data):
    global adjust_heading

    if data.data:
        adjust_heading = False
    else:
        adjust_heading = True

def callback_boot_time(_):
    global adjust_altitude, adjust_heading

    pub_adjust_altitude.publish(adjust_altitude)
    pub_adjust_heading.publish(adjust_heading)

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