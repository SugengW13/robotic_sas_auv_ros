# ROS Package for MAVLink Based AUV

A package to control autonomous underwater vehicle.

## Getting Started

### Requirements

- [git](https://git-scm.com/downloads)
- [ros-noetic-desktop-full](http://wiki.ros.org/noetic/Installation)
- [pymavlink](https://www.ardusub.com/developers/pymavlink.html)

### Setup

To create your own workspace follow this tutorial, [Installing and Configuring your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Clone this repository into your ROS workspace.

```bash
cd <your_ros_workspace>/src
git clone https://github.com/SugengW13/techsas_auv_ros
cd ..
catkin_make
```

To provide object detection feature, we will use [yolov5](https://github.com/ultralytics/yolov5) algorithm that developed by [ultralytics](https://ultralytics.com).

First, you have to make your own ROS package, [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage), and make sure to name it as <b>yolov5</b>.

Afer that, clone the [yolov5](https://github.com/ultralytics/yolov5) repository into your <b>yolov5</b> ROS package.

```bash
cd <your_ros_workspace>/src
git clone https://github.com/ultralytics/yolov5
cd ..
catkin_make
```

To integrate <b>yolov5</b> as an object detection algorithm into this repository, you have to copy this code [node_object_detection.py](https://github.com/SugengW13/YOLO-V5-ROS/blob/main/node_object_detection.py) into your <b>yolov5</b> ROS package.

Then, change the permisson of the code.

```bash
cd <your_ros_workspace>/src/yolov5
sudo chmod +x node_object_detection.py
cd ../..
catkin_make
```

Setup Finish.

### Run

To run the package, follow these steps.

#### 1st Terminal

```bash
roscore
```

#### 2nd Terminal

```bash
roslaunch techsas techsas.launch
```

### Troubleshoot

Make sure to pay attention to the Pixhawk connectivity.

### References
- [ros](https://www.ros.org/)
- [yolov5](https://github.com/ultralytics/yolov5)
