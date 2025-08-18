# hsr_joint_state_2_joint_trajectory

This package is to manipulate HSR via joint_state_publisher.

## Prerequisites

- **xacro**
- **joint_state_publisher_gui**
- **hsrb_description**
- **hsrb_meshes**

For reference
```bash
$ sudo apt install ros-$ROS_DISTRO-xacro
$ sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
$ cd ~/ros2_ws/src
$ git clone --single-branch --branch humble https://github.com/hsr-project/hsrb_description.git
$ git clone --single-branch --branch humble https://github.com/hsr-project/hsrb_meshes.git
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source ~/ros2_ws/install/setup.bash
```

## How To Use

```bash
$ ros2 launch hsr_joint_state_2_joint_trajectory hsr_joint_state_2_joint_trajectory_launch.xml
```
