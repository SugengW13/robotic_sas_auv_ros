#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Bool, Int16, Float32

class Subscriber(object):
    def __init__(self):
        # system
        self.boot_time = 0

        # guidance
        self.search_object = True
        self.pwm_forward = 1500

        # publisher
        self.pub_pwm_forward = rospy.Publisher('pwm_forward', Int16, queue_size=10)

        # subscriber
        rospy.Subscriber('search_object', Bool, self.callback_search_object)
        rospy.Subscriber('distance_from_bottom', Int16, self.callback_distance_from_bottom)
        rospy.Subscriber('boot_time', Float32, self.callback_boot_time)

    def callback_search_object(self, data):
        self.search_object = data.data

        if self.search_object:
            self.pwm_forward = 1500

    def callback_distance_from_bottom(self, data):
        distance = data.data
        
        if distance >= 150:
            self.pwm_forward = int(np.interp(distance, (150, 450), (1550, 1600)))
        elif distance <= 100:
            self.pwm_forward = int(np.interp(distance, (0, 100), (1400, 1450)))
        else:
            self.pwm_forward = 1500
        
        if self.pwm_forward >= 1600:
            self.pwm_forward = 1600

    def callback_boot_time(self, data):
        boot_time = data.data

        self.pub_pwm_forward.publish(self.pwm_forward)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_control_forward', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()