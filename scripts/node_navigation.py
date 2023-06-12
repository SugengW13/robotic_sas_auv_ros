#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def callback_heading(data):
    print(data.data)

def main():
    rospy.init_node('node_navigation', anonymous=True)

    rospy.Subscriber('/rosserial/heading', Float32, callback_heading)

    rospy.spin()

if __name__ == '__main__':
    main()