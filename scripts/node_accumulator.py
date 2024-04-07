#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from robotic_sas_auv_ros.msg import ArduinoSensor, Sensor, BoundingBox, ObjectDetection, Heading
from nav_msgs.msg import Odometry
from detection_msgs.msg import BoundingBoxes

class Subscriber():
    def __init__(self):
        self.is_pre_calibrating = False

        self.offset_roll = 0
        self.offset_pitch = 0
        self.offset_yaw = 0

        self.offset_depth = 0

        self.sensor = Sensor()
        self.object_detection = ObjectDetection()

        # Publisher
        self.pub_sensor = rospy.Publisher('sensor', Sensor, queue_size=10)
        self.pub_object_detection = rospy.Publisher('object_detection', ObjectDetection, queue_size=10)

        # Subscriber
        rospy.Subscriber('is_start', Bool, self.callback_is_start)
        rospy.Subscriber('/imu', Imu, self.callback_imu)
        rospy.Subscriber('/witmotion/heading', Heading, self.callback_heading)
        rospy.Subscriber('/camera/odom/sample', Odometry, self.callback_odometry)
        rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.callback_bounding_boxes)
        rospy.Subscriber('/rosserial/sensor', ArduinoSensor, self.callback_arduino_sensor)

    def pre_calibrate(self):
        # Set offset values in order to set the initial sensor value to zero
        self.offset_roll = self.sensor.roll
        self.offset_pitch = self.sensor.pitch
        self.offset_yaw = self.sensor.yaw
        self.offset_depth = self.sensor.depth

    def get_offset(self, offset):
        # Return the given offset if the pre calibration is complete
        return offset if not self.is_pre_calibrating else 0

    # Collect Arduino Sensor Data
    def callback_arduino_sensor(self, data: ArduinoSensor):
        self.sensor.depth = data.depth - self.get_offset(self.offset_depth)

    # Collect Realsense Position Data
    def callback_odometry(self, data: Odometry):
        self.sensor.pos_x = data.pose.pose.position.x
        self.sensor.pos_y = data.pose.pose.position.y
        self.sensor.pos_z = data.pose.pose.position.z

    # Collect IMU Data
    def callback_imu(self, data: Imu):
        self.sensor.roll = round(data.orientation.x, 3) - self.get_offset(self.offset_roll)
        self.sensor.pitch = round(data.orientation.y, 3) - self.get_offset(self.offset_pitch)

    # Collect Heading Date
    def callback_heading(self, data: Heading):
        self.sensor.yaw = round(data.yaw) - self.get_offset(self.offset_yaw)

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
        self.is_pre_calibrating = not data.data

        # Condition for pre calibrating
        if data.data:
            self.pub_sensor.publish(self.sensor)
            self.pub_object_detection.publish(self.object_detection)
        else:
            self.pre_calibrate()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_accumulator', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()