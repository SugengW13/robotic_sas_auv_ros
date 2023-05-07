#!/usr/bin/env python3

import time
from pymavlink import mavutil

from PyMavlink import ROV

def main(rov: ROV):
    rov.arm()

    rov.setHeading(360, 4)

    rov.disarm()

if __name__ == '__main__':
    # master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    
    master.wait_heartbeat()

    rov = ROV(master)

    main(rov)