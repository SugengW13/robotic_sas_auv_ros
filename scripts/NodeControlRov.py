#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Int16, Bool
from pymavlink import mavutil
from PyMavlink import ROV

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

rov = ROV(master)

rov.arm()
           
# rov.setMode('ALT_HOLD')

is_object_detected = False
is_start = False
boot_time = 0
pwm_lateral = 0
pwm_forward = 0

def callback_is_object_detected(data):
    global is_object_detected, boot_time

    is_object_detected = data.data

    if not is_object_detected:
        if boot_time % 6 == 1 or boot_time % 6 == 2 or boot_time % 6 == 3:
            print('Forward')
            rov.setRcValue(4, 1500)
            rov.setRcValue(5, 1550)
        else:
            print('Yaw')
            rov.setRcValue(5, 1500)
            rov.setRcValue(4, 1450)

def callback_pwm_lateral(data):
    global pwm_lateral

    if not is_object_detected:
        return

    pwm_lateral = data.data

    if is_start:
        rov.setRcValue(6, pwm_lateral)

def callback_pwm_forward(data):
    global pwm_forward
    
    if not is_object_detected:
        return

    pwm_forward = data.data

    if is_start:
        rov.setRcValue(5, pwm_forward)

def callback_is_start(data):
    global is_start
    is_start = data.data

def callback_release_gripper(data):
    open_gripper = data.data

    if open_gripper:
        print('Open Gripper')

        time.sleep(3)

        print('Surfacing')

        time.sleep(3)

        rov.disarm()

def callback_boot_time(data):
    global boot_time

    boot_time = data.data

def main():
    rospy.init_node('node_control_rov', anonymous=True)
    
    rospy.Subscriber('is_object_detected', Bool, callback_is_object_detected)
    rospy.Subscriber('pwm_lateral', Int16, callback_pwm_lateral)
    rospy.Subscriber('pwm_forward', Int16, callback_pwm_forward)
    rospy.Subscriber('release_gripper', Bool, callback_release_gripper)
    rospy.Subscriber('boot_time', Int16, callback_boot_time)
    rospy.Subscriber('is_start', Bool, callback_is_start)

    rospy.spin()

if __name__ == '__main__':
    rov.arm()

    main()