#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Int16

def main():
    startTime = time.time()
    pubBootTime = rospy.Publisher('boot_time', Int16, queue_size=10)

    rospy.init_node('node_boot_time', anonymous=True)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        currentTime = time.time()

        bootTime = int(currentTime - startTime)

        pubBootTime.publish(bootTime)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass