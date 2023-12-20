#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int16, Float32

def main():
    duration = 10
    start_time = time.time()

    pwm_thr_1 = 0
    pwm_thr_2 = 0
    pwm_thr_3 = 0
    pwm_thr_4 = 0
    pwm_thr_5 = 0
    pwm_thr_6 = 0
    pwm_thr_7 = 0
    pwm_thr_8 = 0

    pub_pwm_thr_1 = rospy.Publisher('pwm_thr_1', Int16, queue_size=10)
    pub_pwm_thr_2 = rospy.Publisher('pwm_thr_2', Int16, queue_size=10)
    pub_pwm_thr_3 = rospy.Publisher('pwm_thr_3', Int16, queue_size=10)
    pub_pwm_thr_4 = rospy.Publisher('pwm_thr_4', Int16, queue_size=10)
    pub_pwm_thr_5 = rospy.Publisher('pwm_thr_5', Int16, queue_size=10)
    pub_pwm_thr_6 = rospy.Publisher('pwm_thr_6', Int16, queue_size=10)
    pub_pwm_thr_7 = rospy.Publisher('pwm_thr_7', Int16, queue_size=10)
    pub_pwm_thr_8 = rospy.Publisher('pwm_thr_8', Int16, queue_size=10)


    rospy.init_node('node_control', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and time.time() - start_time < duration if duration else True:
        pub_pwm_thr_1.publish(pwm_thr_1)
        pub_pwm_thr_2.publish(pwm_thr_2)
        pub_pwm_thr_3.publish(pwm_thr_3)
        pub_pwm_thr_4.publish(pwm_thr_4)
        pub_pwm_thr_5.publish(pwm_thr_5)
        pub_pwm_thr_6.publish(pwm_thr_6)
        pub_pwm_thr_7.publish(pwm_thr_7)
        pub_pwm_thr_8.publish(pwm_thr_8)
        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass