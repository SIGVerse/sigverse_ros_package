# sigverse_turtlebot3_open_manipulator

This package includes sample ROS nodes for TurtleBot3 equipped with an arm.

## Prerequisites

### SLAM
- **slam-toolbox**

### recognize_pointed_direction
- **yolo_ros**

### grasping_auto
- **yolo_ros**
- **moveit**

## How To Use

teleop_key
```bash
$ ros2 launch sigverse_turtlebot3 teleop_key_launch.xml
```

SLAM
```bash
$ ros2 launch sigverse_turtlebot3 slam.launch
```

recognize_pointed_direction
```bash
$ ros2 launch sigverse_turtlebot3 recognize_pointed_direction.launch
```

grasping_auto
```bash
$ ros2 launch sigverse_turtlebot3 grasping_auto_launch.xml
```

