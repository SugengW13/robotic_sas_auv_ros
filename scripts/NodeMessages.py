#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Int16, Float32
from pymavlink import mavutil
from PyMavlink import ROV

time.sleep(3)

while True:
    try:
        master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
        # master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        break
    except:
        continue

master.wait_heartbeat()

rov = ROV(master)

def main():
    pub_base_mode = rospy.Publisher('base_mode', Int16, queue_size=10)
    pub_custom_mode = rospy.Publisher('custom_mode', Int16, queue_size=10)
    pub_altitude = rospy.Publisher('altitude', Float32, queue_size=120)
    pub_heading = rospy.Publisher('heading', Int16, queue_size=10)

    rospy.init_node('node_messeges', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            heart_beat = rov.getDataMessage('HEARTBEAT')
            base_mode = heart_beat.base_mode
            custom_mode = heart_beat.custom_mode

            ahrs2 = rov.getDataMessage('AHRS2')
            altitude = round(ahrs2.altitude, 4)

            vfr_hud = rov.getDataMessage('VFR_HUD')
            heading = vfr_hud.heading

            pub_base_mode.publish(base_mode)
            pub_custom_mode.publish(custom_mode)
            pub_altitude.publish(altitude)
            pub_heading.publish(heading)

            rate.sleep()
        except:
            continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass