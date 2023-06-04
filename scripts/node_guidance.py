#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Bool, Int16

class Subscriber():
    def __init__(self):
        self.window_width = 640
        self.window_height = 480

        self.is_stablilizing = False
        self.start_stable_time = 0
        self.stable_duration = 0
        
        self.boot_time = 0
        self.is_object_detected = False
        self.center_x = None
        self.center_y = None
        self.pwm_lateral = 1500
        self.pwm_forward = 1500
        self.is_arm = False
        self.is_alt_hold = False
        self.release_gripper = False
        
        self.pub_pwm_lateral = rospy.Publisher('pwm_lateral', Int16, queue_size=10)
        self.pub_pwm_forward = rospy.Publisher('pwm_forward', Int16, queue_size=10)
        self.pub_is_arm = rospy.Publisher('is_arm', Bool, queue_size=10)
        self.pub_is_alt_hold = rospy.Publisher('is_alt_hold', Bool, queue_size=10)
        self.pub_release_gripper = rospy.Publisher('release_gripper', Bool, queue_size=10)
        
        rospy.Subscriber('is_object_detected', Bool, self.callback_is_object_detected)
        rospy.Subscriber('center_x', Int16, self.callback_center_x)
        rospy.Subscriber('center_y', Int16, self.callback_center_y)
        rospy.Subscriber('base_mode', Int16, self.callback_base_mode)
        rospy.Subscriber('custom_mode', Int16, self.callback_custom_mode)
        rospy.Subscriber('boot_time', Int16, self.callback_boot_time)

    def is_stable_position(self, distance):
        if 100 <= distance <= 150:
            if not self.is_stabilizing:
                self.start_stable_time = self.boot_time

            self.is_stabilizing = True
        else:
            self.stable_duration = 0
            self.is_stabilizing = False

        if self.is_stabilizing:
            self.stable_duration = self.boot_time - self.start_stable_time

        print(self.stable_duration)

        if self.stable_duration >= 3:
            print('Release Gripper')
            self.release_gripper = True

    def callback_is_object_detected(self, data):
        self.is_object_detected = data.data

    def callback_center_x(self, data):
        self.center_x = data.data

        if not self.is_object_detected:
            return

        distance_from_center = self.center_x - self.window_width / 2
        
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
        self.center_y = data.data

        if not self.is_object_detected:
            return

        distance_from_bottom = self.window_height - self.center_y

        # Distance  =>  Min = 150        Max = 450
        # PWM       =>  Min = 1550      Max = 1600

        # Distance  =>  Min = 100        Max = 0
        # PWM       =>  Min = 1450      Max = 1400 
        
        if self.pwm_lateral != 1500:
            self.pwm_forward = 1500
            return

        if distance_from_bottom >= 150:
            self.pwm_forward = int(np.interp(distance_from_bottom, (150, 450), (1550, 1600)))
            self.is_stable_position(distance_from_bottom)
        elif distance_from_bottom <= 100:
            self.pwm_forward = int(np.interp(distance_from_bottom, (0, 100), (1400, 1450)))
            self.is_stable_position(distance_from_bottom)
        else:
            self.pwm_forward = 1500
            self.is_stable_position(distance_from_bottom)

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

    def callback_boot_time(self, data):
        self.boot_time = data.data

        if self.is_object_detected:
            self.pub_pwm_lateral.publish(self.pwm_lateral)
            self.pub_pwm_forward.publish(self.pwm_forward)
        
        self.pub_is_arm.publish(self.is_arm)
        self.pub_is_alt_hold.publish(self.is_alt_hold)
        self.pub_release_gripper.publish(self.release_gripper)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_guidance', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()