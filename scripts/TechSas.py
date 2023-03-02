import time
from pymavlink import mavutil
from PyMavlink import PyMavlink

# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

def main(rov: PyMavlink):
    rov.getDataMessage('VFR_HUD')

if __name__ == '__main__':
    pymavlink = PyMavlink(master)

    main(pymavlink)