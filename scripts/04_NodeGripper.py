#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def callbackX(data):
    rospy.loginfo(data)
    

def callbackY(data):
    rospy.loginfo(data)
    
def main():
    rospy.init_node('node_gripper', anonymous=True)

    rospy.Subscriber("coordinate_x", Float32, callbackX)
    rospy.Subscriber("coordinate_y", Float32, callbackX)

    rospy.spin()

if __name__ == '__main__':
    main()