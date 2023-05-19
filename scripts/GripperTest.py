import time

from PyMavlink import ROV
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

rov = ROV(master)

rov.arm()

rov.closeGripper()

time.sleep(5)

rov.closeGripper()