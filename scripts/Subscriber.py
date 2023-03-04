#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import String

constant = 0.5
# distance * k = pwm - 1500

def calculateHypotenuse(a, b):
    return math.sqrt(a**2 + b**2)

def getLateralPwm(distance_x):
    if distance_x >= 200:
        return 1650
    elif distance_x <= -200:
        return 1350
    else:
        return distance_x * constant + 1550

def getThrottlePwm(distance_y):
    if distance_y >= 200:
        return 1650
    elif distance_y <= -200:
        return 1350
    else:
        return distance_y * constant + 1550

def callback(data):
    rospy.loginfo(data)
    
    try:
        strCoordinate = data.data
        arrCoordinate = strCoordinate.split()
        
        x, y = float(arrCoordinate[0]), float(arrCoordinate[1])

        distance = calculateHypotenuse(320 - x, 240 - y)

        if distance <= 25:
            print('Go Forward')
            pwmFwd = 1600
        else:
            print('Track Object')
            pwmLat = getLateralPwm(x - 320)
            pwmThr = getThrottlePwm(240 - y)
    except:
        print('SEARCH OBJECT')

def subscriber():
    rospy.init_node('subscriber_coordinate', anonymous=True)

    rospy.Subscriber("coordinate", String, callback)

    rospy.spin()

if __name__ == '__main__':
    subscriber()