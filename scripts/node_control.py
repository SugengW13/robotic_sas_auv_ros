#!/usr/bin/env python3

import rospy
import time
from simple_pid import PID
from std_msgs.msg import Bool
from robotic_sas_auv_ros.msg import Error
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

        # Publisher
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
        self.pwm_actuator.thruster_5 = 1500 + pwm
        self.pwm_actuator.thruster_6 = 1500 - pwm
        self.pwm_actuator.thruster_7 = 1500 + pwm
        self.pwm_actuator.thruster_8 = 1500 - pwm
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
        self.publish()

    def publish(self):
        self.pub_pwm_actuator.publish(self.pwm_actuator)

class Subscriber():
    def __init__(self):
        self.is_start = False
        self.start_time = None

        self.movement = Movement()
        self.movement.stop()

        self.pid_roll = PID(5, 0, 0)
        self.pid_pitch = PID(5, 0, 0)
        self.pid_yaw = PID(5, 0, 0)
        self.pid_heave = PID(1, 0, 0)

        self.pwm_roll = 0
        self.pwm_pitch = 0
        self.pwm_yaw = 0
        self.pwm_heave = 0

        # Subscriber
        rospy.Subscriber('error', Error, self.callback_error)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)
    
    def stabilize_roll(self, error):
        self.pwm_roll = self.pid_roll(error)

        if self.pwm_roll >= 50:
            self.pwm_roll = 50
        elif self.pwm_roll <= -50:
            self.pwm_roll = -50

        rospy.loginfo('PWM Roll %s' % self.pwm_roll)
        self.movement.roll(self.pwm_roll)

    def stabilize_pitch(self, error):
        self.pwm_pitch = self.pid_pitch(error)

        if self.pwm_pitch >= 50:
            self.pwm_pitch = 50
        elif self.pwm_pitch <= -50:
            self.pwm_pitch = -50

        rospy.loginfo('PWM Pitch %s' % self.pwm_pitch)
        self.movement.pitch(self.pwm_pitch)


    def stabilize_yaw(self, error):
        self.pwm_yaw = self.pid_yaw(error)

        if self.pwm_yaw >= 50:
            self.pwm_yaw = 50
        elif self.pwm_yaw <= -50:
            self.pwm_yaw = -50

        rospy.loginfo('PWM Yaw %s' % self.pwm_yaw)
        self.movement.yaw(self.pwm_yaw)

    def stabilize_depth(self, error):
        self.pwm_heave = self.pid_heave(error)
        
        if self.pwm_heave >= 50:
            self.pwm_heave = 50
        elif self.pwm_heave <= -50:
            self.pwm_heave = -50

        rospy.loginfo('PWM Heave %s' % self.pwm_heave)
        self.movement.heave(self.pwm_heave)

    def callback_error(self, data):
        # self.stabilize_roll(data.roll)
        # self.stabilize_pitch(data.pitch)
        self.stabilize_yaw(data.yaw)
        self.stabilize_depth(data.depth)

    def callback_is_start(self, data):
        if data.data:
            if not self.is_start:
                self.start_time = time.time()
                self.is_start = True
        else:
            self.movement.stop()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_control', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()