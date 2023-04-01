#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16

class Subscriber(object):
    def __init__(self):
        self.is_center = False
        self.pub = rospy.Publisher('pwm_forward', Int16, queue_size=10)

        rospy.Subscriber('x_coordinate', Float32, self.callback_x)
        rospy.Subscriber('upper_coordinate', Float32, self.callback_upper)
    
    def callback_x(self, data):
        distance_from_center = data.data - 320
        
        if -50 < distance_from_center < 50:
            self.is_center = True
        else:
            self.is_center = False

    def callback_upper(self, data):
        if self.is_center:
            upper_coordinate = data.data
            
            if upper_coordinate <= 240:
                self.pub.publish(1600)
            else:
                self.pub.publish(1500)
    
    def loop(self):
        rospy.spin()

def main():
    rospy.init_node('node_calculate_forward', anonymous=True)

    subscriber = Subscriber()
    subscriber.loop()

if __name__ == '__main__':
    main()