import time
from PyMavlink import ROV
from pymavlink import mavutil

# Normal Front Angle
frontAngle = 0

# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

def main():
    bootTime = time.time()

    rov = ROV(master)

    rov.arm()

    rov.setMode('ALT_HOLD')

    rov.setDepth(bootTime, -0.5)

    time.sleep(1)    

    for _ in 5:
        rov.setRcValue(5, 1600)
        time.sleep(5)
        rov.setRcValue(5, 1500)

        rov.setHeading(bootTime, frontAngle + 90)

    rov.setDepth(bootTime, 0)

    rov.disarm()

if __name__ == '__main__':
    main()