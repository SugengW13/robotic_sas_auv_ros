#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Bool, Float32

class Subscriber():
    def __init__ (self):
        param_port = rospy.get_param('port_hwt')
        param_baud = rospy.get_param('/witmotion/baud_hwt')

        self.ser = serial.Serial(param_port, param_baud)

        # Publisher
        self.pub_heading = rospy.Publisher('/hwt/heading', Float32, queue_size=10)

        # Subscriber
        rospy.Subscriber('/arduino/is_start', Bool, self.callback_is_start)

    def publish_heading(self):
        self.ser.write(bytes([0x33]))

        data = self.ser.readline()
        rospy.loginfo(data)
        data_str = data.decode('utf-8')
        
        for item in data_str.split(','):
            if 'Yaw' in item:
                yaw = float(item.split(':')[1])
                converted_yaw = (yaw / 32768) * 180.0
                self.pub_heading.publish(converted_yaw)

    def callback_is_start(self, data: Bool):
        if data.data:
            rospy.loginfo(data.data)
            self.publish_heading()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_hwt', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()
