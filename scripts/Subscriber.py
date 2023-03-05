#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import String
from pymavlink import mavutil

from PyMavlink import PyMavlink

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
# master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()

rov = PyMavlink(master)

constant = 0.5
# distance * k = pwm - 1500

def getLateralPwm(distance_x):
    print(distance_x)
    if distance_x >= 200:
        return 1650
    elif distance_x <= -200:
        return 1350
    else:
        if distance_x > 0:
            return distance_x * constant + 1550
        elif distance_x < 0:
            return distance_x * constant + 1450

def getThrottlePwm(distance_y):
    if distance_y >= 200:
        return 1650
    elif distance_y <= -200:
        return 1350
    else:
        return distance_y * constant + 1550

def callback(data):
    rospy.loginfo((data.data))
    
    if data.data != '':
        strCoordinate = data.data
        arrCoordinate = strCoordinate.split()
        
        x, y, upper_position = float(arrCoordinate[0]), float(arrCoordinate[1]), float(arrCoordinate[2])
        
        x_from_center = x - 320
        
        if -25 <= x_from_center <= 25:
            if upper_position < 240:
                print('Go Forward')

                rov.setRcValue(5, 1700)
            else:
                print('Release Object')
        else:
            print('Track Object')

            pwmLat = getLateralPwm(x - 320)
            print(int(pwmLat))
            rov.setRcValue(6, int(pwmLat))
    else:
        print('SEARCH OBJECT')

def subscriber():
    rospy.init_node('subscriber_coordinate', anonymous=True)

    rospy.Subscriber("coordinate", String, callback)

    rospy.spin()

if __name__ == '__main__':
    subscriber()