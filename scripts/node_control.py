#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int16, Float32

class Subscriber():
    def __init__(self):
        self.pwm_thr_1 = 1500
        self.pwm_thr_2 = 1500
        self.pwm_thr_3 = 1500
        self.pwm_thr_4 = 1500
        self.pwm_thr_5 = 1500
        self.pwm_thr_6 = 1500
        self.pwm_thr_7 = 1500
        self.pwm_thr_8 = 1500

        # Publisher
        self.pub_pwm_thr_1 = rospy.Publisher('pwm_thr_1', Float32, queue_size=10)
        self.pub_pwm_thr_2 = rospy.Publisher('pwm_thr_2', Float32, queue_size=10)
        self.pub_pwm_thr_3 = rospy.Publisher('pwm_thr_3', Float32, queue_size=10)
        self.pub_pwm_thr_4 = rospy.Publisher('pwm_thr_4', Float32, queue_size=10)
        self.pub_pwm_thr_5 = rospy.Publisher('pwm_thr_5', Float32, queue_size=10)
        self.pub_pwm_thr_6 = rospy.Publisher('pwm_thr_6', Float32, queue_size=10)
        self.pub_pwm_thr_7 = rospy.Publisher('pwm_thr_7', Float32, queue_size=10)
        self.pub_pwm_thr_8 = rospy.Publisher('pwm_thr_8', Float32, queue_size=10)

        # Subscriber
        rospy.Subscriber('error_roll', Int16, self.callback_error_roll)
        rospy.Subscriber('error_pitch', Int16, self.callback_error_pitch)
        rospy.Subscriber('error_yaw', Int16, self.callback_error_yaw)
        rospy.Subscriber('error_depth', Float32, self.callback_error_depth)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)

    def callback_error_roll(self, data):
        # Kalkulasi PID
        print('Calculate PID Roll')

    def callback_error_pitch(self, data):
        # Kalkulasi PID
        print('Calculate PID Pitch')

    def callback_error_yaw(self, data):
        # Kalkulasi PID
        print('Calculate PID Yaw')

    def callback_error_depth(self, data):
        # Kalkulasi PID
        print('Calculate PID Depth')

    def callback_is_start(self, data):
        if data.data:
            self.pub_pwm_thr_1.publish(self.pwm_thr_1)
            self.pub_pwm_thr_2.publish(self.pwm_thr_2)
            self.pub_pwm_thr_3.publish(self.pwm_thr_3)
            self.pub_pwm_thr_4.publish(self.pwm_thr_4)
            self.pub_pwm_thr_5.publish(self.pwm_thr_5)
            self.pub_pwm_thr_6.publish(self.pwm_thr_6)
            self.pub_pwm_thr_7.publish(self.pwm_thr_7)
            self.pub_pwm_thr_8.publish(self.pwm_thr_8)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_control', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()