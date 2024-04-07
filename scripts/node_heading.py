#!/usr/bin/env python3

import rospy
import serial
from robotic_sas_auv_ros.msg import Heading

def main():
    param_port = rospy.get_param('/witmotion/port_heading')
    param_baud = rospy.get_param('/witmotion/baud_heading')
    param_rate = rospy.get_param('/witmotion/rate_heading')

    ser = serial.Serial(param_port, param_baud)

    pub_heading = rospy.Publisher('/witmotion/heading', Heading, queue_size=10)

    heading = Heading()

    rospy.init_node('node_heading', anonymous=True)

    rate = rospy.Rate(param_rate)
    
    try:
        while not rospy.is_shutdown():
            data_str = ser.readline().decode('utf-8').strip()
            rospy.loginfo(data_str)

            for data in data_str.split(','):
                data_name, data_value = data.split(':')
                
                if data_name == 'Magx':
                    heading.mag_x = int(data_value)
                if data_name == 'Magy':
                    heading.mag_y = int(data_value)
                if data_name == 'Magz':
                    heading.mag_z = int(data_value)
                if data_name == 'Yaw':
                    heading.yaw = float(data_value)

                pub_heading.publish(heading)

            rate.sleep()

    except KeyboardInterrupt:
        ser.close()

if __name__ == "__main__":
    main()
