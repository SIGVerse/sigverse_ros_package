#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class HsrJointState2JointTrajectory
{
public:

  HsrJointState2JointTrajectory();
  int run();

private:

  static void rosSigintHandler(int sig);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  double getCurrentJointStatesAngle(std::string joint_name);
  void moveArm();
  void moveHead();
  void moveGripper();

  std::map<std::string, double> default_joint_angle_;
  sensor_msgs::JointState current_joint_states_;

  ros::NodeHandle node_handle_;
  ros::Subscriber sub_joint_state_;
  ros::Publisher  pub_arm_trajectory_;
  ros::Publisher  pub_head_trajectory_;
  ros::Publisher  pub_gripper_trajectory_;
};


HsrJointState2JointTrajectory::HsrJointState2JointTrajectory()
{
  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  default_joint_angle_["arm_lift_joint"]   = 0.0;
  default_joint_angle_["arm_flex_joint"]   = 0.0;
  default_joint_angle_["arm_roll_joint"]   = 0.0;
  default_joint_angle_["wrist_flex_joint"] = 0.0;
  default_joint_angle_["wrist_roll_joint"] = 0.0;
  default_joint_angle_["head_pan_joint"]   = 0.0;
  default_joint_angle_["head_tilt_joint"]  = 0.0;
  default_joint_angle_["hand_motor_joint"] = 0.0;

  std::string sub_joint_state_topic_name;
  std::string pub_arm_trajectory_topic_name;
  std::string pub_head_trajectory_topic_name;
  std::string pub_gripper_trajectory_topic_name;

  node_handle_.param<std::string>("sub_joint_state_topic_name",        sub_joint_state_topic_name,        "/joint_states");
  node_handle_.param<std::string>("pub_arm_trajectory_topic_name",     pub_arm_trajectory_topic_name,     "/hsrb/arm_trajectory_controller/command");
  node_handle_.param<std::string>("pub_head_trajectory_topic_name",    pub_head_trajectory_topic_name,    "/hsrb/head_trajectory_controller/command");
  node_handle_.param<std::string>("pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name, "/hsrb/gripper_controller/command");

  sub_joint_state_        = node_handle_.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &HsrJointState2JointTrajectory::jointStateCallback, this);
  pub_arm_trajectory_     = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
  pub_head_trajectory_    = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_head_trajectory_topic_name, 10);
  pub_gripper_trajectory_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);
}


void HsrJointState2JointTrajectory::rosSigintHandler(int sig)
{
  ros::shutdown();
}


void HsrJointState2JointTrajectory::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  current_joint_states_ = *joint_state;
}


double HsrJointState2JointTrajectory::getCurrentJointStatesAngle(std::string joint_name)
{
  if(current_joint_states_.position.empty())
  {
    return default_joint_angle_[joint_name];
  }

  std::vector<std::string>::iterator it =std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(), joint_name);
  if (it != current_joint_states_.name.end())
  {
    int index = std::distance(current_joint_states_.name.begin(), it);
    return current_joint_states_.position[index];
  }
  else
  {
    return default_joint_angle_[joint_name];
  }
}


void HsrJointState2JointTrajectory::moveArm()
{
  std::vector<double> goal_position;
  goal_position.push_back(getCurrentJointStatesAngle("arm_lift_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("arm_flex_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("arm_roll_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("wrist_flex_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("wrist_roll_joint"));

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;
  arm_joint_point.positions = goal_position;
  arm_joint_point.time_from_start = ros::Duration(0.5);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("arm_lift_joint");
  joint_trajectory.joint_names.push_back("arm_flex_joint");
  joint_trajectory.joint_names.push_back("arm_roll_joint");
  joint_trajectory.joint_names.push_back("wrist_flex_joint");
  joint_trajectory.joint_names.push_back("wrist_roll_joint");
  joint_trajectory.points.push_back(arm_joint_point);

  pub_arm_trajectory_.publish(joint_trajectory);
}


void HsrJointState2JointTrajectory::moveHead()
{
  std::vector<double> goal_position;
  goal_position.push_back(getCurrentJointStatesAngle("head_pan_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("head_tilt_joint"));

  trajectory_msgs::JointTrajectoryPoint head_joint_point;
  head_joint_point.positions = goal_position;
  head_joint_point.time_from_start = ros::Duration(0.5);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("head_pan_joint");
  joint_trajectory.joint_names.push_back("head_tilt_joint");
  joint_trajectory.points.push_back(head_joint_point);

  pub_head_trajectory_.publish(joint_trajectory);
}


void HsrJointState2JointTrajectory::moveGripper()
{
  std::vector<double> goal_position;
  goal_position.push_back(getCurrentJointStatesAngle("hand_motor_joint"));

  trajectory_msgs::JointTrajectoryPoint gripper_joint_point;
  gripper_joint_point.positions = goal_position;
  gripper_joint_point.time_from_start = ros::Duration(0.5);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("hand_motor_joint");
  joint_trajectory.points.push_back(gripper_joint_point);

  pub_gripper_trajectory_.publish(joint_trajectory);
}


int HsrJointState2JointTrajectory::run()
{
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    moveArm();
    moveHead();
    moveGripper();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hsr_joint_state_2_joint_trajectory");
  HsrJointState2JointTrajectory hsr_joint_state_2_joint_trajectory;
  return hsr_joint_state_2_joint_trajectory.run();
}
