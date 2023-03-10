#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float32

def callback(data):
    rospy.loginfo(data)

    pub = rospy.Publisher('depth', Float32, queue_size=10)

    rospy.loginfo('Node Depth')

    pub.publish(0)
    
def main():
    rospy.init_node('node_depth', anonymous=True)
    rospy.Subscriber("is_armed", Bool, callback)

    rospy.spin()

if __name__ == '__main__':
    main()