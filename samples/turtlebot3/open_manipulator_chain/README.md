## Under construction

## Description

This is a teleoperation tool for turtlebot3 with open manipulator chain.

## Setup

### Install rosbridge-server

Please see below.  
http://wiki.ros.org/rosbridge_suite

### Install SIGVerse ROS bridge server

Please see below.  
https://github.com/SIGVerse/ros_package/tree/master/sigverse_ros_bridge

### Install sample program

```bash:
$ cd ~/catkin_ws/src
$ git clone https://github.com/SIGVerse/ros_package.git
$ cd ..
$ catkin_make
```

## How to use

```bash:
$ roslaunch rosbridge_server rosbridge_websocket.launch
$ rosrun sigverse_ros_bridge sigverse_ros_bridge
$ rosrun sigverse_turtlebot3_open_manipulator_chain teleop_key
```


