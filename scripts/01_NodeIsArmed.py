#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int16

def callback(data):
    pub = rospy.Publisher('is_armed', Bool, queue_size=10)

    is_armed = False
    base_mode = data.data

    if base_mode == 81:
        is_armed = False
    elif base_mode == 209:
        is_armed = True

    pub.publish(is_armed)

def main():
    rospy.init_node('node_is_armed', anonymous=True)

    rospy.Subscriber("base_mode", Int16, callback)

    rospy.spin()

if __name__ == '__main__':
    main()