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
        self.pwm_lateral = 1500

        # publisher
        self.pub_pwm_lateral = rospy.Publisher('pwm_lateral', Int16, queue_size=10)

        # subscriber
        rospy.Subscriber('search_object', Bool, self.callback_search_object)
        rospy.Subscriber('distance_from_center', Int16, self.callback_distance_from_center)
        rospy.Subscriber('boot_time', Float32, self.callback_boot_time)

    def callback_search_object(self, data):
        self.search_object = data.data

    def callback_distance_from_center(self, data):
        distance = data.data

        if self.search_object:
            return
        
        if distance >= 50:
            self.pwm_lateral = int(np.interp(distance, (50, 250), (1550, 1600)))
        elif distance <= -50:
            self.pwm_lateral = int(np.interp(distance, (-250, -50), (1300, 1450)))
        else:
            self.pwm_lateral = 1500
        
        if self.pwm_lateral >= 1600:
            self.pwm_lateral = 1600
        elif self.pwm_lateral <= 1300:
            self.pwm_lateral = 1300

    def callback_boot_time(self, data):
        boot_time = data.data

        self.pub_pwm_lateral.publish(self.pwm_lateral)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_control_lateral', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()