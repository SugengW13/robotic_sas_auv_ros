#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

class Subscriber(object):
    def __init__(self):
        self.is_arm = False
        self.is_alt_hold = False

        rospy.Subscriber('base_mode', Int16, self.callback_base_mode)
        rospy.Subscriber('custom_mode', Int16, self.callback_custom_mode)
        rospy.Subscriber('boot_time', Int16, self.callback_boot_time)

    def callback_base_mode(self, data):    
        if data.data == 209:
            self.is_arm = True

    def callback_custom_mode(self, data):
        if data.data == 2:
            self.is_alt_hold = True
    
    def callback_boot_time(self, data):
        bootTime = data.data

        if bootTime%5 == 0:
            print(data.data)
            

def main():
    rospy.init_node('node_set_depth', anonymous=True)
    print('asdf')

    Subscriber()

    rospy.spin()

if __name__ == '__main__':
    main()