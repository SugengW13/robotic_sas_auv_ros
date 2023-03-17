#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def callback(data):
    rospy.loginfo(data)

    rospy.loginfo('PWM Forward')
    
def main():
    rospy.init_node('node_forward', anonymous=True)

    rospy.Subscriber("pwm_forward", Float32, callback)

    rospy.spin()

if __name__ == '__main__':
    main()