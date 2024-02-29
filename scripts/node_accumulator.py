#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from robotic_sas_auv_ros.msg import Sensor
from robotic_sas_auv_ros.msg import ArduinoSensor
from robotic_sas_auv_ros.msg import Orientation

class Subscriber():
    def __init__(self):
        # Publisher
        self.sensor = Sensor()

        self.rate = rospy.Rate(10)

        self.pub_sensor = rospy.Publisher('sensor', Sensor, queue_size=10)

        # Subscriber
        rospy.Subscriber('is_start', Bool, self.callback_is_start)
        rospy.Subscriber('/arduino/sensor', ArduinoSensor, self.callback_sensor)
        rospy.Subscriber('/imu', Imu, self.callback_imu)

    # def convert_to_degree(self, orientation):
    #     return int((orientation * -180) % 360)

    def callback_sensor(self, data):
        self.sensor.depth = data.depth

    def callback_imu(self, data):
        self.sensor.x = round(data.orientation.x, 3)
        self.sensor.y = round(data.orientation.y, 3)
        self.sensor.z = round(data.orientation.z, 3)

    def callback_is_start(self, data):
        if data.data:
            self.pub_sensor.publish(self.sensor)

        self.rate.sleep()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_accumulator', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()