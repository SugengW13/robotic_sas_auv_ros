#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Bool
from robotic_sas_auv_ros.msg import SetPoint

class Subscriber():
    def __init__ (self):
        self.is_start = False
        self.start_time = None

        self.set_point = SetPoint()
        self.set_point.roll = 0
        self.set_point.pitch = 0
        self.set_point.yaw = 15
        self.set_point.depth = -0.6

        self.rate = rospy.Rate(10)

        self.param_duration = rospy.get_param('/nuc/duration')

        self.pub_is_start = rospy.Publisher('is_start', Bool, queue_size=10)
        self.pub_set_point = rospy.Publisher('set_point', SetPoint, queue_size=10)

        rospy.Subscriber('/arduino/is_start', Bool, self.callback_is_start)

    def callback_is_start(self, data):
        if data.data:
            if not self.is_start:
                self.start_time = time.time()
                self.is_start = True
            
            if time.time() - self.start_time < self.param_duration if self.param_duration >= 0 else True:
                self.pub_set_point.publish(self.set_point)
                self.pub_is_start.publish(True)
                return
        
        rospy.loginfo('STOP')
        self.pub_is_start.publish(False)
            
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