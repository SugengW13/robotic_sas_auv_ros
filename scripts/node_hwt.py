#!/usr/bin/env python3

import serial
import time
import rospy
from std_msgs.msg import Float32

param_port = rospy.get_param('/witmotion/port_hwt')
param_baud = rospy.get_param('/witmotion/baud_hwt')

rate = rospy.Rate(10)

print("start")
# Open serial connection
ser = serial.Serial(param_port, param_baud)  # Update 'ttyUSB0' with the correct serial port
# time.sleep(2)  # Wait for serial connection to stabilize
print("port")

# Initialize ROS node
rospy.init_node('node_hwt')

# Create publisher
pub = rospy.Publisher('/hwt/heading', Float32, queue_size=10)

try:
    while not rospy.is_shutdown():
        # Request data from the sensor
        ser.write(bytes([0x33]))  # Command to request data

        # Read data from the sensor
        start_time = time.time()
        while ser.in_waiting == 0:
            if time.time() - start_time > 5:  # Timeout after 5 seconds
                print("Timeout waiting for data")
                break

        # Read data from the sensor
        data = ser.readline()  # Read a line of data
        # print(f"Received data: {data}")  # Print the raw data

        # Extract yaw value from the data
        data_str = data.decode('utf-8')  # Convert bytes to string
        for item in data_str.split(','):
            if 'Yaw' in item:
                yaw_str = item.split(':')[1]  # Get the value after 'Yaw:'
                yaw = float(yaw_str)  # Convert the yaw value to float
                converted_yaw = yaw / 32768.0 * 180.0
                print(converted_yaw)
                pub.publish(converted_yaw)  # Publish the yaw value

        rate.sleep()

except rospy.ROSInterruptException:
    pass
finally:
    ser.close()