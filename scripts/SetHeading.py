from pymavlink import mavutil
from PyMavlink import ROV
import time

def getHeading(master):
    message = master.recv_match(type='VFR_HUD', blocking=True)
    return message.heading

def main():
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    master.wait_heartbeat()

    rov = ROV(master)

    bootTime = time.time()

    targetHeading = 90

    while True:
        attitude = rov.getDataMessage('ATTITUDE')
        vfrHud = rov.getDataMessage('VFR_HUD')

        print(attitude)
        print(vfrHud)

if __name__ == '__main__':
    main()