#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from robotic_sas_auv_ros.msg import SetPoint, IsStable, Movement

class Subscriber():
    def __init__ (self):
        self.is_start = False
        self.boot_time = 0
        self.start_time = 0

        self.is_stable = IsStable()
        self.set_point = SetPoint()
        self.movement = Movement()

        self.set_point.roll = 0
        self.set_point.pitch = 0
        self.set_point.yaw = 0
        self.set_point.depth = 0

        self.param_delay = rospy.get_param('/nuc/delay')
        self.param_duration = rospy.get_param('/nuc/duration')

        # Publisher
        self.pub_is_start = rospy.Publisher('is_start', Bool, queue_size=10)
        self.pub_set_point = rospy.Publisher('set_point', SetPoint, queue_size=10)
        self.pub_movement = rospy.Publisher('movement', Movement, queue_size=10)

        # Subscriber
        rospy.Subscriber('/rosserial/is_start', Bool, self.callback_is_start)
        rospy.Subscriber('is_stable', IsStable, self.callback_is_stable)

    def set_depth(self, depth):
        # Change depth set point from the given value
        rospy.loginfo('Set Depth %s', depth)
        self.set_point.depth = depth

    def set_heading(self, heading):
        # Change yaw set point from the given value
        rospy.loginfo('Set Heading %s', heading)
        self.set_point.yaw = heading

    def publish_movement(self, type, pwm):
        # Publish command and pwm value to node control
        rospy.loginfo('Set %s %s', type, pwm)
        self.movement.type = type
        self.movement.pwm = pwm
        self.pub_movement.publish(self.movement)

    def is_in_range(self, start_time, end_time):
        return (self.boot_time > start_time + self.param_delay and end_time is None) or (start_time + self.param_delay) < self.boot_time < (end_time + self.param_delay)

    def start_auv(self):
        # Declares mission planning

        # Stop AUV when the timer reaches 27 secs since pre calibration
        if self.is_in_range(27, None):
            self.stop_auv()
            return

        # Start AUV mission
        self.pub_set_point.publish(self.set_point)
        self.pub_is_start.publish(True)

        # Set mission using timer, is_in_range(start_time, end_time)
        if self.is_in_range(1, 3):
            # Set target depth
            self.set_depth(0.5)

        if self.is_in_range(4, 14):
            # Surge with the speed of 200, publish_movement(command, pwm)
            self.publish_movement('SURGE', 200)

        if self.is_in_range(15, 20):
            # Set target heading
            self.set_heading(180)

        if self.is_in_range(21, 26):
            # Surge with the speed of 200, publish_movement(command, pwm)
            self.publish_movement('SURGE', 200)

    def stop_auv(self):
        # Stop AUV
        rospy.loginfo('STOP')
        self.pub_is_start.publish(False)

    # Collect IsStable Data
    def callback_is_stable(self, data: IsStable):
        self.is_stable = data

    def callback_is_start(self, data: Bool):
        if data.data:
            # Generate boot time
            self.boot_time = rospy.get_time() - self.start_time
            
            # Wait for a secs to tell other nodes (accumulator & control) to calibrate
            if self.boot_time < self.param_delay:
                rospy.loginfo('STARTING...')
                self.pub_is_start.publish(False)
                return

            # Set start time
            if not self.is_start:
                self.start_time = rospy.get_time()
                self.is_start = True

            # Timer condition
            if self.boot_time < self.param_duration if self.param_duration >= 0 else True:
                self.start_auv()
            else:
                self.stop_auv()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_guidance', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass