import time
from pymavlink import mavutil
from PyMavlink import PyMavlink

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

def main(pm: PyMavlink):
    pm.armDisarm()
    
    time.sleep(1)

    pm.arm()

    time.sleep(1)

    pm.disarm()

    time.sleep(1)

    pm.setMode('MANUAL')

    time.sleep(1)

    pm.setRcValue(5, 1550)

    time.sleep(1)

    pm.setRcValue(5, 1500)

    time.sleep(1)

    pm.setHeading(360, 4)

if __name__ == '__main__':
    pymavlink = PyMavlink(master)

    main(pymavlink)