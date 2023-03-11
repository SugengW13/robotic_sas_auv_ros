#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool

def callbackX(data):
    rospy.loginfo('Gripper X')
    

def callbackY(data):
    rospy.loginfo('Gripper Y')
    
def main():
    rospy.init_node('node_gripper', anonymous=True)
    pub = rospy.Publisher('is_success', Bool, queue_size=10)
    rospy.Subscriber("coordinate_x", Float32, callbackX)
    rospy.Subscriber("coordinate_y", Float32, callbackX)

    rospy.loginfo(True)
    pub.publish(True)
    
    rospy.spin()

if __name__ == '__main__':
    main()