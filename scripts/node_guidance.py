#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Bool
from robotic_sas_auv_ros.msg import SetPoint

def main():
    duration = False
    start_time = time.time()

    set_point = SetPoint()
    set_point.roll = 1500
    set_point.pitch = 1500
    set_point.yaw = 1500
    set_point.depth = 1500

    pub_is_start = rospy.Publisher('is_start', Bool, queue_size=10)
    pub_set_point = rospy.Publisher('set_point', SetPoint, queue_size=10)

    rospy.init_node('node_guidance', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and time.time() - start_time < duration if duration else True:
        pub_is_start.publish(True)
        pub_set_point.publish(set_point)
        
        rate.sleep()

    pub_is_start.publish(False)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass