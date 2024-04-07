#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from robotic_sas_auv_ros.msg import Error, Actuator, Movement

class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.proportional = 0
        self.integral = 0
        self.derivative = 0

        self.start_time = rospy.get_time()
        self.last_time = 0

        self.last_error = 0

    def __call__(self, error):
        dt = rospy.get_time() - (self.last_time if self.last_time is not None else self.start_time)

        d_error = error - self.last_error

        self.proportional = self.kp * error
        self.integral += self.ki * error * dt
        self.derivative = self.kd * d_error / dt

        self.last_error = error
        self.last_time = rospy.get_time()

        return self.proportional + self.integral + self.derivative

class ThrusterMovement():
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

    def surge_sway_yaw(self, pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4):
        self.pwm_actuator.thruster_1 = pwm_thruster_1
        self.pwm_actuator.thruster_2 = pwm_thruster_2
        self.pwm_actuator.thruster_3 = pwm_thruster_3
        self.pwm_actuator.thruster_4 = pwm_thruster_4

    def heave_roll_pitch(self, pwm_thruster_5, pwm_thruster_6, pwm_thruster_7, pwm_thruster_8):
        self.pwm_actuator.thruster_5 = pwm_thruster_5
        self.pwm_actuator.thruster_6 = pwm_thruster_6
        self.pwm_actuator.thruster_7 = pwm_thruster_7
        self.pwm_actuator.thruster_8 = pwm_thruster_8

    def boost(self, pwm):
        self.pwm_actuator.thruster_9 = 1500 + pwm
        self.pwm_actuator.thruster_10 = 1500 + pwm

    def surge(self, pwm):
        self.pwm_actuator.thruster_1 = 1500 + pwm
        self.pwm_actuator.thruster_2 = 1500 + pwm
        self.pwm_actuator.thruster_3 = 1500 + pwm
        self.pwm_actuator.thruster_4 = 1500 - pwm

    def sway(self, pwm):
        self.pwm_actuator.thruster_1 = 1500 + pwm
        self.pwm_actuator.thruster_2 = 1500 - pwm
        self.pwm_actuator.thruster_3 = 1500 - pwm
        self.pwm_actuator.thruster_4 = 1500 - pwm

    def yaw(self, pwm):
        self.pwm_actuator.thruster_1 = 1500 - pwm
        self.pwm_actuator.thruster_2 = 1500 + pwm
        self.pwm_actuator.thruster_3 = 1500 - pwm
        self.pwm_actuator.thruster_4 = 1500 + pwm

    def heave(self, pwm):
        self.pwm_actuator.thruster_5 = 1500 - pwm
        self.pwm_actuator.thruster_6 = 1500 + pwm
        self.pwm_actuator.thruster_7 = 1500 + pwm
        self.pwm_actuator.thruster_8 = 1500 + pwm

    def roll(self, pwm):
        self.pwm_actuator.thruster_5 = 1500 + pwm
        self.pwm_actuator.thruster_6 = 1500 - pwm
        self.pwm_actuator.thruster_7 = 1500 - pwm
        self.pwm_actuator.thruster_8 = 1500 + pwm

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
        self.pwm_actuator.thruster_9 = 1500
        self.pwm_actuator.thruster_10 = 1500

    def publish(self):
        self.pub_pwm_actuator.publish(self.pwm_actuator)

class Subscriber():
    def __init__(self):
        self.error = Error()

        self.movement = ThrusterMovement()
        self.movement.stop()

        self.param_arming_duration = rospy.get_param('/nuc/arming_duration')

        self.pid_heave = PID(1000, 0, 0)
        self.pid_roll = PID(1000, 0, 0)
        self.pid_pitch = PID(1000, 0, 0)
        self.pid_yaw = PID(1, 0, 0)

        self.start_time = 0
        self.is_armed = False
        self.is_pre_calibrating = False

        self.offset_roll = 0
        self.offset_pitch = 0
        self.offset_yaw = 0

        self.offset_surge = 0
        self.offset_sway = 0
        self.offset_heave = 0

        self.pwm_roll = 0
        self.pwm_pitch = 0
        self.pwm_yaw = 0

        self.pwm_surge = 0
        self.pwm_sway = 0
        self.pwm_heave = 0

        # Subscriber
        rospy.Subscriber('error', Error, self.callback_error)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)
        rospy.Subscriber('movement', Movement, self.callback_movement)

    def constrain(self, value, _min, _max):
        return min(max(value, _min), _max)

    def pre_calibrate(self):
        # Set offset values in order to set the initial pwm values to zero
        self.offset_roll = self.pwm_roll
        self.offset_pitch = self.pwm_pitch
        self.offset_yaw = self.pwm_yaw

        self.offset_surge = self.pwm_surge
        self.offset_sway = self.pwm_sway
        self.offset_heave = self.pwm_heave

    def get_offset(self, offset):
        # Return the given offset if the pre calibration is complete
        return offset if not self.is_pre_calibrating else 0

    # Calculate surge, sway & yaw movement
    def surge_sway_yaw(self):
        min_pwm = 1200
        max_pwm = 1800

        pwm_thruster_1 = self.constrain(1500 + (self.pwm_surge - self.get_offset(self.offset_surge)) + (self.pwm_sway - self.get_offset(self.offset_sway)) - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
        pwm_thruster_2 = self.constrain(1500 + (self.pwm_surge - self.get_offset(self.offset_surge)) - (self.pwm_sway - self.get_offset(self.offset_sway)) + (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
        pwm_thruster_3 = self.constrain(1500 + (self.pwm_surge - self.get_offset(self.offset_surge)) - (self.pwm_sway - self.get_offset(self.offset_sway)) - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
        pwm_thruster_4 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)) - (self.pwm_sway - self.get_offset(self.offset_sway)) - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)

        self.movement.surge_sway_yaw(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4)

    # Calculate heave, roll & pitch movement
    def heave_roll_pitch(self):
        min_pwm = 1200
        max_pwm = 1800
        
        pwm_thruster_5 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) + (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_6 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) - (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_7 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) + (self.pwm_roll - self.get_offset(self.offset_roll)) - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
        pwm_thruster_8 = self.constrain(1500 + (self.pwm_heave - self.get_offset(self.offset_heave)) - (self.pwm_roll - self.get_offset(self.offset_roll)) - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm) 
            
        self.movement.heave_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    # Stabilize roll, pitch, yaw and depth using PID control
    def stabilize_roll(self, error):
        self.pwm_roll = self.pid_roll(error)

    def stabilize_pitch(self, error):
        self.pwm_pitch = self.pid_pitch(error)

    def stabilize_yaw(self, error):
        self.pwm_yaw = self.pid_yaw(error)

    def stabilize_depth(self, error):
        self.pwm_heave = self.pid_heave(error)

    # Collect Error Data
    def callback_error(self, data: Error):
        self.error = data
        self.stabilize_roll(data.roll)
        self.stabilize_pitch(data.pitch)    
        self.stabilize_depth(data.depth)
        self.stabilize_yaw(data.yaw)

    # Collect Movement Data
    def callback_movement(self, data: Movement):
        if data.type == 'BOOST':
            self.movement.boost()
        if data.type == 'SURGE':
            self.pwm_surge = data.pwm
        if data.type == 'SWAY':
            self.pwm_sway = data.pwm
        if data.type == 'YAW':
            self.pwm_yaw = data.pwm
        if data.type == 'HEAVE':
            self.pwm_heave = data.pwm
        if data.type == 'ROLL':
            self.pwm_roll = data.pwm
        if data.type == 'PITCH':
            self.pwm_pitch = data.pwm
        if data.type == 'STOP':
            self.movement.stop()

    def stabilize(self):
        self.heave_roll_pitch()
        self.surge_sway_yaw()

    def callback_is_start(self, data: Bool):
        self.is_pre_calibrating = not data.data

        # Wait for a secs to await sensor value changes / spikes after pre calibrating
        if not self.is_pre_calibrating and self.start_time + self.param_arming_duration > rospy.get_time():
            rospy.loginfo('READY TO DIVE...')
            return

        # Condition for pre calibrating
        if data.data:
            self.stabilize()
        else:
            self.start_time = rospy.get_time()
            self.pre_calibrate()
            self.movement.stop()

        self.movement.publish()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_control', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()