import serial
import time
import rospy
from std_msgs.msg import Float32


param_port = rospy.get_param('/witmotion/port_hwt')
param_baud = rospy.get_param('/witmotion/baud_hwt')

print("start")
# Open serial connection
ser = serial.Serial(param_port, param_baud)  # Update 'ttyUSB0' with the correct serial port
time.sleep(2)  # Wait for serial connection to stabilize
print("port")

# Initialize ROS node
rospy.init_node('node_hwt')

# Create publisher
pub = rospy.Publisher('/hwt/heading', Float32, queue_size=10)

try:
    while not rospy.is_shutdown():
        print("true")
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
        print(f"Received data: {data}")  # Print the raw data
        print(f"Received data length: {len(data)}")  # Print the length of the received data
        if len(data) >= 8:  # Changed from 9 to 8
            # Extract Euler angles from the received data
            yaw = (data[1] << 8) | data[0]
            pub.publish(yaw)

except rospy.ROSInterruptException:
    pass
finally:
    ser.close()