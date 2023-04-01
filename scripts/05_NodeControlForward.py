#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from pymavlink import mavutil

from PyMavlink import ROV

def callback(data, rov: ROV):
    pwm = data.data

    rov.setRcValue(5, pwm)
    
def main(rov: ROV):
    rospy.init_node('node_forward', anonymous=True)

    rospy.Subscriber("pwm_forward", Int16, callback, rov)

    rospy.spin()

if __name__ == '__main__':
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

    rov = ROV(master)

    main(rov)