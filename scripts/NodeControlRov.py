#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Int16, Float32
from pymavlink import mavutil
from PyMavlink import ROV

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

rov = ROV(master)

def main():
    rospy.init_node('node_control_rov', anonymous=True)

    rov.arm()
    
    rov.setMode('ALT_HOLD')
    
    time.sleep(1)

    rov.setDepth(0, -0.5)

    time.sleep(1)

    startTime = time.time()

    while True:
        bootTime = int(time.time() - startTime)
        
        if bootTime <= 30:
            rov.setRcValue(5, 1600)

            if bootTime > 0 and bootTime%5 == 0:
                rov.setRcValue(5, 1550)
        else:
            break

        time.sleep(1)

    rov.setDepth(0, 0)

    time.sleep(1)

    rov.disarm()

if __name__ == '__main__':
    main()