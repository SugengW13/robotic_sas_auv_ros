#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Bool, Int16, Float32

# Params
window_width = 640
window_height = 480
target_altitude = -0.5
target_heading = 90

# Data to Publish
pwm_lateral = 0
pwm_forward = 0
is_arm = False
is_alt_hold = False
is_unstable_altitude = False
is_unstable_heading = False

# Publisher
pub_pwm_lateral = rospy.Publisher('pwm_lateral', Int16, queue_size=10)
pub_pwm_forward = rospy.Publisher('pwm_forward', Int16, queue_size=10)
pub_is_arm = rospy.Publisher('is_arm', Bool, queue_size=10)
pub_is_alt_hold = rospy.Publisher('is_alt_hold', Bool, queue_size=10)
pub_is_unstable_altitude = rospy.Publisher('is_unstable_altitude', Bool, queue_size=10)
pub_is_unstable_heading = rospy.Publisher('is_unstable_heading', Bool, queue_size=10)

def calculate_pwm_lateral(x):
    distance_from_center = x - window_width / 2
    pwm = 0

    # Min Pos at +- 25, Max at += 300
    # Min PWM at += 50, Max at += 150
    
    if distance_from_center > 0:
        pwm = int(np.interp(distance_from_center, (25, 300), (50, 150)))
    elif distance_from_center < 0:
        pwm = int(np.interp(distance_from_center, (-300, -25), (-150, -50)))

    return pwm

def calculate_pwm_forward(y):
    distance_from_bottom = window_height - y
    pwm = 0

    # Min Pos at 0, Max at 400
    # Min PWM at 0, Max at 200
    
    if distance_from_bottom > 0:
        pwm = int(np.interp(distance_from_bottom, (0, 400), (0, 200)))

    return pwm

def calculate_angle_range(degree, tolerance):
    lower_bound = degree - tolerance

    if lower_bound < 0:
        lower_bound += 360

    upper_bound = degree + tolerance

    if upper_bound > 360:
        upper_bound -= 360

    return [lower_bound, upper_bound]

def callback_center_x(data):
    global pwm_lateral

    center_x = data.data
    pwm_lateral = calculate_pwm_lateral(center_x)

def callback_center_y(data):
    global pwm_forward

    center_y = data.data
    pwm_forward = calculate_pwm_forward(center_y)

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
    
    pub_pwm_lateral.publish(pwm_lateral)
    pub_pwm_forward.publish(pwm_forward)
    pub_is_arm.publish(is_arm)
    pub_is_alt_hold.publish(is_alt_hold)
    pub_is_unstable_altitude.publish(is_unstable_altitude)
    pub_is_unstable_heading.publish(is_unstable_heading)

def main():
    rospy.init_node('node_guidance', anonymous=True)

    rospy.Subscriber('center_x', Int16, callback_center_x)
    rospy.Subscriber('center_y', Int16, callback_center_y)
    rospy.Subscriber('base_mode', Int16, callback_base_mode)
    rospy.Subscriber('custom_mode', Int16, callback_custom_mode)
    rospy.Subscriber('altitude', Float32, callback_altitude)
    rospy.Subscriber('heading', Int16, callback_heading)
    rospy.Subscriber('boot_time', Int16, callback_boot_time)

    rospy.spin()

if __name__ == '__main__':
    main()