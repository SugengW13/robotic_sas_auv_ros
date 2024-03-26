#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Bool
from robotic_sas_auv_ros.msg import SetPoint, IsStable, Movement

class Subscriber():
    def __init__ (self):
        self.is_start = False
        self.boot_time = None
        self.start_time = None

        self.is_stable = IsStable()
        self.set_point = SetPoint()
        self.movement = Movement()

        self.set_point.roll = 0
        self.set_point.pitch = 0
        self.set_point.yaw = 0.9
        self.set_point.depth = 0

        self.param_delay = rospy.get_param('/nuc/delay')
        self.param_duration = rospy.get_param('/nuc/duration')

        param_rate = rospy.get_param('/nuc/rate')
        self.rate = rospy.Rate(param_rate)

        # Publisher
        self.pub_is_start = rospy.Publisher('is_start', Bool, queue_size=10)
        self.pub_set_point = rospy.Publisher('set_point', SetPoint, queue_size=10)
        self.pub_movement = rospy.Publisher('movement', Movement, queue_size=10)

        # Subscriber
        rospy.Subscriber('/arduino/is_start', Bool, self.callback_is_start)
        rospy.Subscriber('is_stable', IsStable, self.callback_is_stable)

    def publish_movement(self, type, pwm):
        self.movement.type = type
        self.movement.pwm = pwm
        self.pub_movement.publish(self.movement)

    def start_movement(self, start_time, end_time):
        rospy.loginfo('MOVE')

    def start_auv(self, boot_time):
        if boot_time <= self.param_delay:
            rospy.loginfo('STARTING...')
            return

        self.pub_set_point.publish(self.set_point)
        self.pub_is_start.publish(True)

        self.publish_movement('SURGE', 400)

    def stop_auv(self):
        rospy.loginfo('STOP')
        self.pub_is_start.publish(False)

    # Collect IsStable Data
    def callback_is_stable(self, data: IsStable):
        self.is_stable = data

    def callback_is_start(self, data: Bool):
        if data.data:
            if not self.is_start:
                self.start_time = time.time()
                self.is_start = True

            if time.time() - self.start_time < self.param_duration if self.param_duration >= 0 else True:
                self.start_auv(time.time() - self.start_time)
            else:
                self.stop_auv()

        self.rate.sleep()

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