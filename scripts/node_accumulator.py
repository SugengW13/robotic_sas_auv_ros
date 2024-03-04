#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from robotic_sas_auv_ros.msg import Sensor
from robotic_sas_auv_ros.msg import ArduinoSensor
from nav_msgs.msg import Odometry

class Subscriber():
    def __init__(self):
        self.sensor = Sensor()
        self.rate = rospy.Rate(10)

        # Publisher
        self.pub_sensor = rospy.Publisher('sensor', Sensor, queue_size=10)

        # Subscriber
        rospy.Subscriber('is_start', Bool, self.callback_is_start)
        rospy.Subscriber('/arduino/sensor', ArduinoSensor, self.callback_arduino_sensor)
        rospy.Subscriber('/camera/odom/sample', Odometry, self.callback_odometry)
        rospy.Subscriber('/imu', Imu, self.callback_imu)

    # Collect Arduino Sensor Data
    def callback_arduino_sensor(self, data: ArduinoSensor):
        self.sensor.depth = data.depth

    # Collect Realsense Position Data
    def callback_odometry(self, data: Odometry):
        self.sensor.pos_x = data.pose.pose.position.x
        self.sensor.pos_y = data.pose.pose.position.y
        self.sensor.pos_z = data.pose.pose.position.z

    # Collect WitMotion Data
    def callback_imu(self, data: Imu):
        self.sensor.roll = round(data.orientation.x, 3)
        self.sensor.pitch = round(data.orientation.y, 3)
        self.sensor.yaw = round(data.orientation.z, 3)

    def callback_is_start(self, data: Bool):
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
