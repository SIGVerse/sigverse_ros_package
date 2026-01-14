# Setup

## Install ROS 2 Humble Hawksbill

Please see the link below.  
https://github.com/SIGVerse/sigverse_unity_project/wiki/Tutorial-using-ROS#install-ros-2-humble-hawksbill

## Install Mongo C Driver

Please see the link below.  
https://github.com/SIGVerse/sigverse_unity_project/wiki/Tutorial-using-ROS#install-mongo-c-driver

## Install Mongo C++ Driver

Please see the link below.  
https://github.com/SIGVerse/sigverse_unity_project/wiki/Tutorial-using-ROS#install-mongo-cpp-driver

## Install SIGVerse ROS Bridge

```bash:
cd ~/ros2_ws/src
git clone https://github.com/SIGVerse/sigverse_ros_package.git
cd ~/ros2_ws/
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```


# How to use

```bash:
$ ros2 run sigverse_ros_bridge sigverse_ros_bridge
```

Default port number is 50001.  
If you need change the port number, please pass the argument.

```bash
$ ros2 run sigverse_ros_bridge sigverse_ros_bridge 12345
```


