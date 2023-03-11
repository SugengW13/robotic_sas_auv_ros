#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def callback(data):
    rospy.loginfo('DISARM')
    
def main():
    rospy.init_node('node_disarm', anonymous=True)

    rospy.Subscriber("is_surfaced", Bool, callback)

    rospy.spin()

if __name__ == '__main__':
    main()