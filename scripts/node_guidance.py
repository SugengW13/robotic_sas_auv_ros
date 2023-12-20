#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int16, Float32

def main():
    duration = 5
    start_time = time.time()

    target_roll = 0
    target_pitch = 0
    target_yaw = 0
    target_depth = 0


    pub_target_roll = rospy.Publisher('target_roll', Int16, queue_size=10)
    pub_target_pitch = rospy.Publisher('target_pitch', Int16, queue_size=10)
    pub_target_yaw = rospy.Publisher('target_yaw', Int16, queue_size=10)
    pub_target_depth = rospy.Publisher('target_depth', Float32, queue_size=10)

    rospy.init_node('node_guidance', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and time.time() - start_time < duration if duration else True:
        pub_target_roll.publish(target_roll)
        pub_target_pitch.publish(target_pitch)
        pub_target_yaw.publish(target_yaw)
        pub_target_depth.publish(target_depth)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass