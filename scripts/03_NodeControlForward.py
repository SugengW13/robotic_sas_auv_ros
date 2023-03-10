#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def callback(data):
    rospy.loginfo(data)

    pub = rospy.Publisher('pwm_forward', Float32, queue_size=10)

    rospy.loginfo('Control Forward')
    
    pub.publish(0)
    
def main():
    rospy.init_node('node_control_forward', anonymous=True)

    rospy.Subscriber("coordinate_y", Float32, callback)

    rospy.spin()

if __name__ == '__main__':
    main()