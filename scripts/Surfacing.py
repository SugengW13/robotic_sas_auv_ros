import time
from pymavlink import mavutil
from PyMavlink import ROV

# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

rov = ROV(master)

def main():
    rov.arm()

    rov.setRcValue(3, 1400)
    # rov.setRcValue(5, 1600)
    time.sleep(5)
    rov.setRcValue(3, 1500)
    # rov.setRcValue(5, 1500)

    rov.disarm()

if __name__ == '__main__':
    main()