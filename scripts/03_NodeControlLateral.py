#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def callback(data):
    rospy.loginfo(data)

    pub = rospy.Publisher('pwm_lateral', Float32, queue_size=10)

    rospy.loginfo('Control Lateral')
    
    pub.publish(0)
    
def main():
    rospy.init_node('node_depth', anonymous=True)

    rospy.Subscriber("coordinate_x", Float32, callback)

    rospy.spin()

if __name__ == '__main__':
    main()