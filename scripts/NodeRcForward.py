#!/usr/bin/env python3

import time
import rospy
from pymavlink import mavutil
from std_msgs.msg import Bool

from PyMavlink import ROV

# master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

rov = ROV(master)

# class Subscriber(object):
#     def __init__(self):
#         self.isArm = False
#         self.pwmForward = 1550

#         self.sub = rospy.Subscriber('is_arm', Bool, self.callbackIsArm)

#     def callbackIsArm(self, data):
#         print(self.isArm)
#         self.isArm = data.data

#         self.setPwmForward()

#     def setPwmForward(self):
#         if self.isArm:
#             print('GASS')

#             rov.setRcValue(5, 1550)
#             time.sleep(5)
#             rov.setRcValue(5, 1500)

#             print('DONE DONE DONE DONE')

#             self.sub.unregister()

def main():
    # rospy.init_node('node_rc_forward', anonymous=True)

    # Subscriber()

    # rospy.spin()

    rov.setRcValue(5, 1550)
    time.sleep(5)
    rov.setRcValue(5, 1500)

if __name__ == '__main__':
    time.sleep(5)
    main()