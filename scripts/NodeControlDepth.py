#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Float32, Int16

depthTarget = -0.5
lastError = 0.0
kp = 160
kd = 35

pubPwmDepth = rospy.Publisher('pwm_depth', Int16, queue_size=10)

def calculatePid(depth):
    global lastError

    currentError = depth - depthTarget

    diffError = currentError - lastError

    pid = int((kp * currentError) + (kd * diffError))

    lastError = currentError

    return pid

def calculatePwm(pid):
    pwm = pid + 1500

    if pwm >= 1550:
        pwm = 1550
    if pwm <= 1450:
        pwm = 1450

    return pwm

def callback(data):
    depth = data.data

    pid = calculatePid(depth)
    pwm = calculatePwm(pid)
    print(pwm)
    pubPwmDepth.publish(pwm)

def main():
    rospy.init_node('node_control_depth', anonymous=True)

    rospy.Subscriber('altitude', Float32, callback)

    rospy.spin()

if __name__ == '__main__':
    time.sleep(3)

    main()