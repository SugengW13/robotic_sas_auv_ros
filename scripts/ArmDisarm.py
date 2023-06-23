from PyMavlink import ROV
from pymavlink import mavutil

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

rov = ROV(master)

rov.armDisarm()