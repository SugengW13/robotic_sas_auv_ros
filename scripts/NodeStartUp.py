#!/usr/bin/env python3

import rospy
from pymavlink import mavutil

from PyMavlink import ROV

def main(rov: ROV):
    rospy.init_node('node_start_up', anonymous=True)

    rov.arm()
    rov.setMode('ALT_HOLD')

if __name__ == '__main__':
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    
    master.wait_heartbeat()

    rov = ROV(master)
    
    try:
        main(rov)
    except rospy.ROSInterruptException:
        pass