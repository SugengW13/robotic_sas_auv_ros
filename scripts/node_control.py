#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from robotic_sas_auv_ros.msg import Error
from robotic_sas_auv_ros.msg import Actuator

class Subscriber():
    def __init__(self):
        self.pwm_thruster = Actuator()
        self.pwm_thruster.thruster_1 = 1500
        self.pwm_thruster.thruster_2 = 1500
        self.pwm_thruster.thruster_3 = 1500
        self.pwm_thruster.thruster_4 = 1500
        self.pwm_thruster.thruster_5 = 1500
        self.pwm_thruster.thruster_6 = 1500
        self.pwm_thruster.thruster_7 = 1500
        self.pwm_thruster.thruster_8 = 1500

        # Publisher
        self.pub_pwm_thruster = rospy.Publisher('pwm_thruster', Actuator, queue_size=10)

        # Subscriber
        rospy.Subscriber('error', Error, self.callback_error)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)

    def callback_error(self, data):
        # rospy.loginfo(data)
        pass

    def callback_is_start(self, data):
        if data.data:
            self.pub_pwm_thruster.publish(self.pwm_thruster)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_control', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()