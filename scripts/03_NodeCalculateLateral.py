#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16

def get_pwm(distance):
    if distance > 0:
        if distance >= 200:
            return 1600
        else:
            return int((distance + 4600) / 3)
    elif distance < 0:
        if distance <= -200:
            return 1400
        else:
            return int((distance + 4400) / 3)

def calculate_pwm_lateral(x):
    distance_from_center = x - 320

    if -50 < distance_from_center < 50:
        return None
    else:
        return get_pwm(distance_from_center)

def callback(data):
    pub = rospy.Publisher('pwm_lateral', Int16, queue_size=10)
    
    pwm = calculate_pwm_lateral(data.data)
    
    if pwm != None:
        pub.publish(pwm)
    
def main():
    rospy.init_node('node_calculate_lateral', anonymous=True)

    rospy.Subscriber("x_coordinate", Float32, callback)

    rospy.spin()

if __name__ == '__main__':
    main()