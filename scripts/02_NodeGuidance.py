#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, String

def callback(data):
    rospy.loginfo(data)

    pubX = rospy.Publisher('coordinate_x', Float32, queue_size=10)
    pubY = rospy.Publisher('coordinate_y', Float32, queue_size=10)

    rospy.loginfo('Node Depth')
    
    pubX.publish(0)
    pubY.publish(0)
    
def main():
    rospy.init_node('node_depth', anonymous=True)

    rospy.Subscriber("coordinate", String, callback)
    rospy.Subscriber("depth", Float32, callback)
    rospy.Subscriber("position", String, callback)

    rospy.spin()

if __name__ == '__main__':
    main()