#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32

def main():
    print('asdfasdfasdf')
    param_port = rospy.get_param('port_hwt')
    param_baud = rospy.get_param('/witmotion/baud_hwt')

    ser = serial.Serial(param_port, param_baud)

    pub_heading = rospy.Publisher('/hwt/heading', Float32, queue_size=10)

    rospy.init_node('node_hwt', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ser.write(bytes([0x33]))

        data = ser.readline()

        data_str = data.decode('utf-8')

        for item in data_str.split(','):
            if 'Yaw' in item:
                yaw = float(item.split(':')[1])
                converted_yaw = (yaw / 32768) * 180.0
                pub_heading.publish(converted_yaw)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
