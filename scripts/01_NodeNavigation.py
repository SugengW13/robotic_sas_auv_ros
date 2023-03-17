#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String

def callback(data):
    rospy.loginfo(data)

    pub = rospy.Publisher('position', String, queue_size=10)
    
    rospy.loginfo('Node Navigation')
    
    pub.publish('Test')
    
def main():
    rospy.init_node('node_navigation', anonymous=True)
    rospy.Subscriber("is_armed", Bool, callback)

    rospy.spin()

if __name__ == '__main__':
    main()