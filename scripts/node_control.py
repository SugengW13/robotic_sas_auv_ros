#!/usr/bin/env python3

import rospy
import time
from simple_pid import PID
from std_msgs.msg import Bool, String
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

        self.pub_pwm_actuator = rospy.Publisher('pwm_actuator', Actuator, queue_size=10)

    def surgeSwayYaw(self, pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4):
        self.pwm_actuator.thruster_1 = pwm_thruster_1
        self.pwm_actuator.thruster_2 = pwm_thruster_2
        self.pwm_actuator.thruster_3 = pwm_thruster_3
        self.pwm_actuator.thruster_4 = pwm_thruster_4

    def heaveRollPitch(self, pwm_thruster_5, pwm_thruster_6, pwm_thruster_7, pwm_thruster_8):
        self.pwm_actuator.thruster_5 = pwm_thruster_5
        self.pwm_actuator.thruster_6 = pwm_thruster_6
        self.pwm_actuator.thruster_7 = pwm_thruster_7
        self.pwm_actuator.thruster_8 = pwm_thruster_8

    def surge(self, pwm):
        self.pwm_actuator.thruster_1 = 1500 + pwm
        self.pwm_actuator.thruster_2 = 1500 + pwm
        self.pwm_actuator.thruster_3 = 1500 + pwm
        self.pwm_actuator.thruster_4 = 1500 + pwm

    def sway(self, pwm):
        self.pwm_actuator.thruster_1 = 1500 - pwm
        self.pwm_actuator.thruster_2 = 1500 + pwm
        self.pwm_actuator.thruster_3 = 1500 + pwm
        self.pwm_actuator.thruster_4 = 1500 - pwm

    def yaw(self, pwm):
        self.pwm_actuator.thruster_1 = 1500 - pwm
        self.pwm_actuator.thruster_2 = 1500 + pwm
        self.pwm_actuator.thruster_3 = 1500 - pwm
        self.pwm_actuator.thruster_4 = 1500 + pwm

    def heave(self, pwm):
        self.pwm_actuator.thruster_5 = 1500 + pwm
        self.pwm_actuator.thruster_6 = 1500 + pwm
        self.pwm_actuator.thruster_7 = 1500 - pwm
        self.pwm_actuator.thruster_8 = 1500 - pwm

    def roll(self, pwm):
        self.pwm_actuator.thruster_5 = 1500 - pwm
        self.pwm_actuator.thruster_6 = 1500 + pwm
        self.pwm_actuator.thruster_7 = 1500 + pwm
        self.pwm_actuator.thruster_8 = 1500 - pwm

    def pitch(self, pwm):
        self.pwm_actuator.thruster_5 = 1500 + pwm
        self.pwm_actuator.thruster_6 = 1500 + pwm
        self.pwm_actuator.thruster_7 = 1500 + pwm
        self.pwm_actuator.thruster_8 = 1500 + pwm

    def stop(self):
        self.pwm_actuator.thruster_1 = 1500
        self.pwm_actuator.thruster_2 = 1500
        self.pwm_actuator.thruster_3 = 1500
        self.pwm_actuator.thruster_4 = 1500
        self.pwm_actuator.thruster_5 = 1500
        self.pwm_actuator.thruster_6 = 1500
        self.pwm_actuator.thruster_7 = 1500
        self.pwm_actuator.thruster_8 = 1500

    def publish(self):
        print('Publish PWM', self.pwm_actuator)
        self.pub_pwm_actuator.publish(self.pwm_actuator)

class Subscriber():
    def __init__(self):
        self.movement = Movement()
        self.movement.stop()

        self.pid_heave = PID(1000, 50, 200)
        self.pid_roll = PID(200, 2, 3)
        self.pid_pitch = PID(200, 0, 1)
        self.pid_yaw = PID(200, 0, 0)

        self.pwm_roll = 0
        self.pwm_pitch = 0
        self.pwm_yaw = 0

        self.pwm_surge = 0
        self.pwm_sway = 0
        self.pwm_heave = 0

        self.rate = rospy.Rate(10)

        # Subscriber
        rospy.Subscriber('error', Error, self.callback_error)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)
        rospy.Subscriber('movement', String, self.callback_movement)
    
    def constrain(self, value):
        return min(max(value, 1250), 1750)

    def surgeSwayYaw(self):
        pwm_thruster_1 = self.constrain(1500 + self.pwm_surge + self.pwm_sway + self.pwm_yaw)
        pwm_thruster_2 = self.constrain(1500 + self.pwm_surge - self.pwm_sway - self.pwm_yaw)
        pwm_thruster_3 = self.constrain(1500 + self.pwm_surge - self.pwm_sway + self.pwm_yaw)
        pwm_thruster_4 = self.constrain(1500 + self.pwm_surge + self.pwm_sway - self.pwm_yaw)
        self.movement.surgeSwayYaw(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4)

    def heaveRollPitch(self):
        pwm_thruster_5 = self.constrain(1500 + self.pwm_heave + self.pwm_roll - self.pwm_pitch)
        pwm_thruster_6 = self.constrain(1500 + self.pwm_heave - self.pwm_roll - self.pwm_pitch)
        pwm_thruster_7 = self.constrain(1500 - self.pwm_heave - self.pwm_roll - self.pwm_pitch)
        pwm_thruster_8 = self.constrain(1500 - self.pwm_heave + self.pwm_roll - self.pwm_pitch)
        self.movement.heaveRollPitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7, pwm_thruster_8)

    def stabilize_roll(self, error):
        self.pwm_roll = self.pid_roll(error)

    def stabilize_pitch(self, error):
        self.pwm_pitch = self.pid_pitch(error)

    def stabilize_yaw(self, error):
        self.pwm_yaw = self.pid_yaw(error)

    def stabilize_depth(self, error):
        self.pwm_heave = self.pid_heave(error)

    def callback_movement(self, data):
        if data.data == 'SURGE':
            self.pwm_surge = 200

    def callback_error(self, data):
        self.stabilize_roll(data.roll)
        self.stabilize_pitch(data.pitch)
        self.stabilize_depth(data.depth)
        self.stabilize_yaw(data.yaw)

    def callback_is_start(self, data):
        if data.data:
            self.heaveRollPitch()

            self.movement.publish()
        else:
            self.movement.stop()

        self.rate.sleep()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_control', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()