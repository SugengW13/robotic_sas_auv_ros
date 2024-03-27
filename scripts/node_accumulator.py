#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Imu
from robotic_sas_auv_ros.msg import ArduinoSensor, Sensor, BoundingBox, ObjectDetection
from nav_msgs.msg import Odometry
from detection_msgs.msg import BoundingBoxes

class Subscriber():
    def __init__(self):
        self.sensor = Sensor()
        self.object_detection = ObjectDetection()

        # Publisher
        self.pub_sensor = rospy.Publisher('sensor', Sensor, queue_size=10)
        self.pub_object_detection = rospy.Publisher('object_detection', ObjectDetection, queue_size=10)

        # Subscriber
        rospy.Subscriber('is_start', Bool, self.callback_is_start)
        rospy.Subscriber('/arduino/sensor', ArduinoSensor, self.callback_arduino_sensor)
        rospy.Subscriber('/camera/odom/sample', Odometry, self.callback_odometry)
        rospy.Subscriber('/imu', Imu, self.callback_imu)
        rospy.Subscriber('/heading', Float32, self.callback_heading)
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.callback_bounding_boxes)

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
        # self.sensor.yaw = round(data.orientation.z, 3)
    
    def callback_heading(self, data):
        self.sensor.yaw = round(data.data)

    # Collect BoundingBoxes Data
    def callback_bounding_boxes(self, data: BoundingBoxes):
        self.object_detection.bounding_boxes = [
            BoundingBox(
                class_name=b_box.Class,
                probability=b_box.probability,
                x_min=b_box.xmin,
                y_min=b_box.ymin,
                x_max=b_box.xmax,
                y_max=b_box.ymax
            ) for b_box in data.bounding_boxes
        ]

        rospy.loginfo(self.object_detection.bounding_boxes)


    def callback_is_start(self, data: Bool):
        if data.data:   
            self.pub_sensor.publish(self.sensor)
            self.pub_object_detection.publish(self.object_detection)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_accumulator', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()
