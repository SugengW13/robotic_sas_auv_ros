#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Bool, Int16, Float32

class Subscriber():
    def __init__(self):
        self.window_width = 640
        self.window_height = 480

        self.pwm_lateral = 1500
        self.pwm_forward = 1500
        self.is_arm = False
        self.is_alt_hold = False

        self.pub_pwm_lateral = rospy.Publisher('pwm_lateral', Int16, queue_size=10)
        self.pub_pwm_forward = rospy.Publisher('pwm_forward', Int16, queue_size=10)
        self.pub_is_arm = rospy.Publisher('is_arm', Bool, queue_size=10)
        self.pub_is_alt_hold = rospy.Publisher('is_alt_hold', Bool, queue_size=10)

        rospy.Subscriber('center_x', Int16, self.callback_center_x)
        rospy.Subscriber('center_y', Int16, self.callback_center_y)
        rospy.Subscriber('base_mode', Int16, self.callback_base_mode)
        rospy.Subscriber('custom_mode', Int16, self.callback_custom_mode)
        rospy.Subscriber('boot_time', Int16, self.callback_boot_time)

    def callback_center_x(self, data):
        distance_from_center = data.data - self.window_width / 2
        
        # Distance  =>  Min += 50           Max += 250
        # PWM       =>  Min 1500 +- 50      Max 1500 += 100

        if distance_from_center >= 50:
            self.pwm_lateral = int(np.interp(distance_from_center, (50, 250), (1550, 1600)))
        elif distance_from_center <= -50:
            self.pwm_lateral = int(np.interp(distance_from_center, (-250, -50), (1300, 1450)))
        else:
            self.pwm_lateral = 1500

        if self.pwm_lateral >= 1600:
            self.pwm_lateral = 1600
        elif self.pwm_lateral <= 1300:
            self.pwm_lateral = 1300

    def callback_center_y(self, data):
        distance_from_bottom = self.window_height - data.data

        # Distance  =>  Min = 0         Max = 400
        # PWM       =>  Min = 1550      Max = 1600
        
        if self.pwm_lateral != 1500:
            self.pwm_forward = 1500
            return

        if distance_from_bottom > 0:
            self.pwm_forward = int(np.interp(distance_from_bottom, (0, 400), (1550, 1600)))
        
        if self.pwm_forward >= 1600:
            self.pwm_forward = 1600
    
    def callback_base_mode(self, data):
        base_mode = data.data

        if base_mode == 209:
            self.is_arm = True
        else:
            self.is_arm = False

    def callback_custom_mode(self, data):
        custom_mode = data.data

        if custom_mode == 2:
            self.is_alt_hold = True
        else:
            self.is_alt_hold = False

    def callback_boot_time(self, _):
        self.pub_pwm_lateral.publish(self.pwm_lateral)
        self.pub_pwm_forward.publish(self.pwm_forward)
        self.pub_is_arm.publish(self.is_arm)
        self.pub_is_alt_hold.publish(self.is_alt_hold)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_guidance', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()