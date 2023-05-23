#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from pymavlink import mavutil
from PyMavlink import ROV

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

rov = ROV(master)

rov.arm()
           
# rov.setMode('ALT_HOLD')

boot_time = 0
pwm_lateral = 0
pwm_forward = 0

def callback_pwm_lateral(data):
    global pwm_lateral

    pwm_lateral = data.data

    if boot_time > 10:
        rov.setRcValue(6, 1500 + pwm_lateral)

def callback_pwm_forward(data):
    global pwm_lateral, pwm_forward

    pwm_forward = data.data

    if boot_time > 10 and pwm_lateral == 0:
        rov.setRcValue(5, 1500 + pwm_forward)

def callback_boot_time(data):
    global boot_time

    boot_time = data.data

def main():
    rospy.init_node('node_control_rov', anonymous=True)

    rospy.Subscriber('pwm_lateral', Int16, callback_pwm_lateral)
    rospy.Subscriber('pwm_forward', Int16, callback_pwm_forward)
    rospy.Subscriber('boot_time', Int16, callback_boot_time)

    rospy.spin()

if __name__ == '__main__':
    rov.arm()

    main()