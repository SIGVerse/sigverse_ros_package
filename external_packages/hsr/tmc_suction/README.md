Overview
=============================
* Provide actions for using the suction functions.
* Can start/stop suction.
* If it is the case that suction starts, returns whether or not something was suctioned up within a timeout.

Developer
================
* Kazuto Murase


Class
================

SuctionServer
-------------
* Action server class for suction functions.

Interfaces
================
### Provided actions ###
tmc_suction/SuctionControlActionGoal : "/suction_control/goal":
- duration timeout : Object suction timeout (If the timeout is reached, then suction stops).
- std_msgs/Bool suction_on: True: suction starts, False: suction stops.

tmc_suction/SuctionControlActionResult : "/suction_control/result":
- actionlib_msgs/GoalStatus status : - actionlib_msgs::GoalStatus::PREEMPTED Action interrupted - actionlib_msgs::GoalStatus::SUCCEEDED Successful completion - actionlib_msgs::GoalStatus::ABORTED Abnormal termination (Object suction timeout) - actionlib_msgs::GoalStatus::REJECTED Goal rejected (When the timeout duration contains a negative number).

### Subscribed topic ###
std_msgs/Bool : "/pressure_sensor_on": Obtain from the device whether or not it is actually suctioning up something.

### Published topic ###
std_msgs/Bool : "/suction_on": Transmit suction start/stop commands to the device.

LICENSE
================
* This software is released under the BSD 3-Clause Clear License, see LICENSE.txt.
