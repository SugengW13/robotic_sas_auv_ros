#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from pymavlink import mavutil

from PyMavlink import ROV

def nodeArm(rov: ROV):
    isArm = False

    rov.arm()

    pub = rospy.Publisher('is_armed', Bool, queue_size=10)
    rospy.init_node('node_arm', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        dataMessage = rov.getDataMessage('HEARTBEAT')
        
        try:
            baseMode = dataMessage.base_mode

            if baseMode == 81:
                isArm = False
            elif baseMode == 209:
                isArm = True
        except:
            isArm = False

        rospy.loginfo(isArm)
        pub.publish()  

        rate.sleep()

if __name__ == '__main__':
    master = mavutil.mavlink_connection("/dev/ttyACM1", baud=115200)
    # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master.wait_heartbeat()

    rov = ROV(master)

    try:
        nodeArm(rov)
    except rospy.ROSInterruptException:
        pass