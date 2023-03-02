import time
from pymavlink import mavutil
from PyMavlink import PyMavlink
from ServoOutput import Gripper

# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

def main(rov: PyMavlink, gripper: Gripper):
    rov.reboot()

if __name__ == '__main__':
    pymavlink = PyMavlink(master)
    gripper = Gripper(master, 1)

    main(pymavlink, gripper)