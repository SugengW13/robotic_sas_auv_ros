#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Bool
from robotic_sas_auv_ros.msg import Actuator

class Movement():
    def __init__(self):
        self.pwm_actuator = Actuator()
        self.pwm_actuator.thruster_1 = 1500
        self.pwm_actuator.thruster_2 = 1500
        self.pwm_actuator.thruster_3 = 1500
        self.pwm_actuator.thruster_4 = 1500
        self.pwm_actuator.thruster_5 = 1500
        self.pwm_actuator.thruster_6 = 1500
        self.pwm_actuator.thruster_7 = 1500
        self.pwm_actuator.thruster_8 = 1500
        self.pwm_actuator.thruster_9 = 1500
        self.pwm_actuator.thruster_10 = 1500

        self.pub_pwm_actuator = rospy.Publisher('pwm_actuator', Actuator, queue_size=10)

    def surge(self, pwm):
        self.pwm_actuator.thruster_1 = 1500 + pwm
        self.pwm_actuator.thruster_2 = 1500 + pwm
        self.pwm_actuator.thruster_3 = 1500 + pwm
        self.pwm_actuator.thruster_4 = 1500 + pwm
        self.publish()

    def sway(self, pwm):
        self.pwm_actuator.thruster_1 = 1500 - pwm
        self.pwm_actuator.thruster_2 = 1500 + pwm
        self.pwm_actuator.thruster_3 = 1500 + pwm
        self.pwm_actuator.thruster_4 = 1500 - pwm
        self.publish()

    def yaw(self, pwm):
        self.pwm_actuator.thruster_1 = 1500 - pwm
        self.pwm_actuator.thruster_2 = 1500 - pwm
        self.pwm_actuator.thruster_3 = 1500 + pwm
        self.pwm_actuator.thruster_4 = 1500 + pwm
        self.publish()

    def heave(self, pwm):
        self.pwm_actuator.thruster_5 = 1500 + pwm
        self.pwm_actuator.thruster_6 = 1500 + pwm
        self.pwm_actuator.thruster_7 = 1500 + pwm
        self.pwm_actuator.thruster_8 = 1500 + pwm
        self.publish()

    def roll(self, pwm):
        self.pwm_actuator.thruster_5 = 1500 - pwm
        self.pwm_actuator.thruster_6 = 1500 + pwm
        self.pwm_actuator.thruster_7 = 1500 - pwm
        self.pwm_actuator.thruster_8 = 1500 + pwm
        self.publish()

    def pitch(self, pwm):
        self.pwm_actuator.thruster_5 = 1500 + pwm
        self.pwm_actuator.thruster_6 = 1500 + pwm
        self.pwm_actuator.thruster_7 = 1500 - pwm
        self.pwm_actuator.thruster_8 = 1500 - pwm
        self.publish()

    def stop(self):
        self.pwm_actuator.thruster_1 = 1500
        self.pwm_actuator.thruster_2 = 1500
        self.pwm_actuator.thruster_3 = 1500
        self.pwm_actuator.thruster_4 = 1500
        self.pwm_actuator.thruster_5 = 1500
        self.pwm_actuator.thruster_6 = 1500
        self.pwm_actuator.thruster_7 = 1500
        self.pwm_actuator.thruster_8 = 1500
        self.pwm_actuator.thruster_9 = 1500
        self.pwm_actuator.thruster_10 = 1500
        self.publish()

    def publish(self):
        self.pub_pwm_actuator.publish(self.pwm_actuator)

class Subscriber():
    def __init__(self):
        self.is_start = False
        self.start_time = None

        self.movement = Movement()

        self.param_pwm = rospy.get_param('/nuc/pwm')
        self.param_duration = rospy.get_param('/nuc/duration')
        self.param_movement = rospy.get_param('/nuc/movement')

        self.rate = rospy.Rate(10)

        rospy.Subscriber('/arduino/is_start', Bool, self.callback_is_start)

    def start_auv(self):
        if self.param_movement == 'surge':
            self.movement.surge(self.param_pwm)
        if self.param_movement == 'sway':
            self.movement.sway(self.param_pwm)
        if self.param_movement == 'yaw':
            self.movement.yaw(self.param_pwm)
        if self.param_movement == 'heave':
            self.movement.heave(self.param_pwm)
        if self.param_movement == 'roll':
            self.movement.roll(self.param_pwm)
        if self.param_movement == 'pitch':
            self.movement.pitch(self.param_pwm)
        if self.param_movement == 'stop':
            self.movement.stop()

    def stop_auv(self):
        self.movement.stop()

    def callback_is_start(self, data):
        if data.data:
            if not self.is_start:
                self.start_time = time.time()
                self.is_start = True

            if time.time() - self.start_time < self.param_duration if self.param_duration >= 0 else True:
                self.start_auv()
            else:
                self.stop_auv()

            self.rate.sleep()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_test_thr', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass