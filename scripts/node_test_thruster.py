#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float32

class Movement():
    def __init__(self):
        self.pub_pwm_thr_1 = rospy.Publisher('pwm_thr_1', Float32, queue_size=10)
        self.pub_pwm_thr_2 = rospy.Publisher('pwm_thr_2', Float32, queue_size=10)
        self.pub_pwm_thr_3 = rospy.Publisher('pwm_thr_3', Float32, queue_size=10)
        self.pub_pwm_thr_4 = rospy.Publisher('pwm_thr_4', Float32, queue_size=10)
        self.pub_pwm_thr_5 = rospy.Publisher('pwm_thr_5', Float32, queue_size=10)
        self.pub_pwm_thr_6 = rospy.Publisher('pwm_thr_6', Float32, queue_size=10)
        self.pub_pwm_thr_7 = rospy.Publisher('pwm_thr_7', Float32, queue_size=10)
        self.pub_pwm_thr_8 = rospy.Publisher('pwm_thr_8', Float32, queue_size=10)

    def surge(self, pwm):
        self.pub_pwm_thr_1.publish(1500 + pwm)
        self.pub_pwm_thr_2.publish(1500 + pwm)
        self.pub_pwm_thr_3.publish(1500 + pwm)
        self.pub_pwm_thr_4.publish(1500 + pwm)

    def sway(self, pwm):
        self.pub_pwm_thr_1.publish(1500 - pwm)
        self.pub_pwm_thr_2.publish(1500 + pwm)
        self.pub_pwm_thr_3.publish(1500 + pwm)
        self.pub_pwm_thr_4.publish(1500 - pwm)

    def yaw(self, pwm):
        self.pub_pwm_thr_1.publish(1500 - pwm)
        self.pub_pwm_thr_2.publish(1500 - pwm)
        self.pub_pwm_thr_3.publish(1500 + pwm)
        self.pub_pwm_thr_4.publish(1500 + pwm)

    def heave(self, pwm):
        self.pub_pwm_thr_5.publish(1500 + pwm)
        self.pub_pwm_thr_6.publish(1500 + pwm)
        self.pub_pwm_thr_7.publish(1500 + pwm)
        self.pub_pwm_thr_8.publish(1500 + pwm)

    def roll(self, pwm):
        self.pub_pwm_thr_5.publish(1500 - pwm)
        self.pub_pwm_thr_6.publish(1500 + pwm)
        self.pub_pwm_thr_7.publish(1500 - pwm)
        self.pub_pwm_thr_8.publish(1500 + pwm)

    def pitch(self, pwm):
        self.pub_pwm_thr_5.publish(1500 + pwm)
        self.pub_pwm_thr_6.publish(1500 + pwm)
        self.pub_pwm_thr_7.publish(1500 - pwm)
        self.pub_pwm_thr_8.publish(1500 - pwm)

    def stop(self):
        self.pub_pwm_thr_1.publish(1500)
        self.pub_pwm_thr_2.publish(1500)
        self.pub_pwm_thr_3.publish(1500)
        self.pub_pwm_thr_4.publish(1500)
        self.pub_pwm_thr_5.publish(1500)
        self.pub_pwm_thr_6.publish(1500)
        self.pub_pwm_thr_7.publish(1500)
        self.pub_pwm_thr_8.publish(1500)

def main():
    duration = 3
    start_time = time.time()

    rospy.init_node('node_test_thr', anonymous=True)

    movement = Movement()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and time.time() - start_time < duration if duration else True:
        rate.sleep()

    movement.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass