#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import String

def calculateHypotenuse(a, b):
    return math.sqrt(a**2 + b**2)


def callback(data):
    rospy.loginfo(data)
    
    try:
        strCoordinate = data.data
        arrCoordinate = strCoordinate.split()
        
        x, y = float(arrCoordinate[0]), float(arrCoordinate[1])

        distance = calculateHypotenuse(320 - x, 240 - y)

        print(distance)

        if distance <= 50:
            print('GO FORWARD')
        else:
            print('TRACK OBJECT')
    except:
        print('SEARCH OBJECT')

def subscriber():
    rospy.init_node('subscriber_coordinate', anonymous=True)

    rospy.Subscriber("coordinate", String, callback)

    rospy.spin()

if __name__ == '__main__':
    subscriber()