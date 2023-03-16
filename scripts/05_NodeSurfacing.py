#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

def callback(data):
    rospy.loginfo(data)


    rospy.loginfo('DISARM')
    
    
def main():
    rospy.init_node('node_surfacing', anonymous=True)
    pub = rospy.Publisher('is_surfaced', Bool, queue_size=10)

    rospy.Subscriber("is_success", Bool, callback)
    pub.publish(False)

    rospy.spin()

if __name__ == '__main__':
    main()