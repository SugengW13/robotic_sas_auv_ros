#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int16

pubIsArm = rospy.Publisher('is_arm', Bool, queue_size=10)
isArm = False

def callback(data):
    global pubIsArm, isArm

    if data.data == 209:
        isArm = True
    else:
        isArm = False
    
    pubIsArm.publish(isArm)

def main():
    rospy.init_node('node_is_arm', anonymous=True)

    rospy.Subscriber('base_mode', Int16, callback)

    rospy.spin()

if __name__ == '__main__':
    main()