#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def callback(data):
    rospy.loginfo(data)
    
def main():
    rospy.init_node('node_camera', anonymous=True)

    rospy.Subscriber("is_armed", Bool, callback)

    rospy.spin()

if __name__ == '__main__':
    main()