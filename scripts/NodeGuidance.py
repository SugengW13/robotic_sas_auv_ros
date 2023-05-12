#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Bool, Int16, Float32

is_arm = False
is_alt_hold = False
is_unstable_altitude = False
is_unstable_heading = False

target_altitude = -0.5
target_heading = 90

pub_is_arm = rospy.Publisher('is_arm', Bool, queue_size=10)
pub_is_alt_hold = rospy.Publisher('is_alt_hold', Bool, queue_size=10)
pub_is_unstable_altitude = rospy.Publisher('is_unstable_altitude', Bool, queue_size=10)
pub_is_unstable_heading = rospy.Publisher('is_unstable_heading', Bool, queue_size=10)

def calculate_angle_range(degree, tolerance):
    lower_bound = degree - tolerance

    if lower_bound < 0:
        lower_bound += 360

    upper_bound = degree + tolerance

    if upper_bound > 360:
        upper_bound -= 360

    return [lower_bound, upper_bound]

def callback_base_mode(data):
    global is_arm

    if data.data == 209:
        is_arm = True

def callback_custom_mode(data):
    global is_alt_hold
    
    if data.data == 2:
        is_alt_hold = True

def callback_altitude(data):
    global is_unstable_altitude, target_altitude

    tolerance_altitude = [ target_altitude - 0.1, target_altitude + 0.1 ]

    if tolerance_altitude[0] <= data.data <= tolerance_altitude[1]:
        is_unstable_altitude = False
    else:
        is_unstable_altitude = True

def callback_heading(data):
    global is_unstable_heading, target_heading

    angle_range = calculate_angle_range(target_heading, 1)

    if angle_range[0] <= data.data <= angle_range[1]:
        is_unstable_heading = False
    else:
        is_unstable_heading = True
        

def callback_boot_time(_):
    global is_arm, is_alt_hold, altitude, heading

    pub_is_arm.publish(is_arm)
    pub_is_alt_hold.publish(is_alt_hold)
    pub_is_unstable_altitude.publish(is_unstable_altitude)
    pub_is_unstable_heading.publish(is_unstable_heading)

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