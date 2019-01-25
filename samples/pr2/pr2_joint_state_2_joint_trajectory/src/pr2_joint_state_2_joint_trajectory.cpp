#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>

class PR2JointState2JointTrajectory
{
public:

  PR2JointState2JointTrajectory();
  int run();

private:

  static void rosSigintHandler(int sig);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  double getCurrentJointStatesAngle(std::string joint_name);
  void moveHead();
  void moveTorso();
  void moveLeftArm();
  void moveRightArm();
  void moveLeftHand();
  void moveRightHand();

  std::map<std::string, double> default_joint_angle_;
  sensor_msgs::JointState current_joint_states_;

  ros::NodeHandle node_handle_;
  ros::Subscriber sub_joint_state_;
  ros::Publisher  pub_head_trajectory_;
  ros::Publisher  pub_torso_trajectory_;
  ros::Publisher  pub_l_arm_trajectory_;
  ros::Publisher  pub_r_arm_trajectory_;
  ros::Publisher  pub_l_hand_command_;
  ros::Publisher  pub_r_hand_command_;
};


PR2JointState2JointTrajectory::PR2JointState2JointTrajectory()
{
  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  default_joint_angle_["head_pan_joint"]           = 0.0;
  default_joint_angle_["head_tilt_joint"]          = 0.0;
  default_joint_angle_["torso_lift_joint"]         = 0.17;
  default_joint_angle_["l_shoulder_pan_joint"]     = 0.0;
  default_joint_angle_["l_shoulder_lift_joint"]    = 0.0;
  default_joint_angle_["l_upper_arm_roll_joint"]   = 0.0;
  default_joint_angle_["l_elbow_flex_joint"]       = -1.14;
  default_joint_angle_["l_forearm_roll_joint"]     = 0.0;
  default_joint_angle_["l_wrist_flex_joint"]       = -1.05;
  default_joint_angle_["l_wrist_roll_joint"]       = 0.0;
  default_joint_angle_["r_shoulder_pan_joint"]     = 0.0;
  default_joint_angle_["r_shoulder_lift_joint"]    = 0.0;
  default_joint_angle_["r_upper_arm_roll_joint"]   = 0.0;
  default_joint_angle_["r_elbow_flex_joint"]       = -1.14;
  default_joint_angle_["r_forearm_roll_joint"]     = 0.0;
  default_joint_angle_["r_wrist_flex_joint"]       = -1.05;
  default_joint_angle_["r_wrist_roll_joint"]       = 0.0;
  default_joint_angle_["l_gripper_l_finger_joint"] = 0.0;
  default_joint_angle_["r_gripper_l_finger_joint"] = 0.0;

  std::string sub_joint_state_topic_name;
  std::string pub_head_trajectory_topic_name;
  std::string pub_torso_trajectory_topic_name;
  std::string pub_l_arm_trajectory_topic_name;
  std::string pub_r_arm_trajectory_topic_name;
  std::string pub_l_hand_topic_name;
  std::string pub_r_hand_topic_name;

  node_handle_.param<std::string>("sub_joint_state_topic_name",        sub_joint_state_topic_name,        "/joint_states");
  node_handle_.param<std::string>("pub_head_trajectory_topic_name",    pub_head_trajectory_topic_name,    "/head_traj_controller/command");
  node_handle_.param<std::string>("pub_torso_trajectory_topic_name",   pub_torso_trajectory_topic_name,   "/torso_controller/command");
  node_handle_.param<std::string>("pub_l_arm_trajectory_topic_name",   pub_l_arm_trajectory_topic_name,   "/l_arm_controller/command");
  node_handle_.param<std::string>("pub_r_arm_trajectory_topic_name",   pub_r_arm_trajectory_topic_name,   "/r_arm_controller/command");
  node_handle_.param<std::string>("pub_l_hand_topic_name",             pub_l_hand_topic_name,             "/l_gripper_controller/command");
  node_handle_.param<std::string>("pub_r_hand_topic_name",             pub_r_hand_topic_name,             "/r_gripper_controller/command");

  sub_joint_state_        = node_handle_.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &PR2JointState2JointTrajectory::jointStateCallback, this);
  pub_head_trajectory_    = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_head_trajectory_topic_name, 10);
  pub_torso_trajectory_   = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_torso_trajectory_topic_name, 10);
  pub_l_arm_trajectory_   = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_l_arm_trajectory_topic_name, 10);
  pub_r_arm_trajectory_   = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_r_arm_trajectory_topic_name, 10);
  pub_l_hand_command_     = node_handle_.advertise<pr2_controllers_msgs::Pr2GripperCommand>(pub_l_hand_topic_name, 10);
  pub_r_hand_command_     = node_handle_.advertise<pr2_controllers_msgs::Pr2GripperCommand>(pub_r_hand_topic_name, 10);
}


void PR2JointState2JointTrajectory::rosSigintHandler(int sig)
{
  ros::shutdown();
}


void PR2JointState2JointTrajectory::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  current_joint_states_ = *joint_state;
}


double PR2JointState2JointTrajectory::getCurrentJointStatesAngle(std::string joint_name)
{
  if(current_joint_states_.position.empty()){
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


void PR2JointState2JointTrajectory::moveHead()
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


void PR2JointState2JointTrajectory::moveTorso()
{
  std::vector<double> goal_position;
  goal_position.push_back(getCurrentJointStatesAngle("torso_lift_joint"));

  trajectory_msgs::JointTrajectoryPoint gripper_joint_point;
  gripper_joint_point.positions = goal_position;
  gripper_joint_point.time_from_start = ros::Duration(0.5);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("torso_lift_joint");
  joint_trajectory.points.push_back(gripper_joint_point);

  pub_torso_trajectory_.publish(joint_trajectory);
}


void PR2JointState2JointTrajectory::moveLeftArm()
{
  std::vector<double> goal_position;
  goal_position.push_back(getCurrentJointStatesAngle("l_shoulder_pan_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("l_shoulder_lift_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("l_upper_arm_roll_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("l_elbow_flex_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("l_forearm_roll_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("l_wrist_flex_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("l_wrist_roll_joint"));

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;
  arm_joint_point.positions = goal_position;
  arm_joint_point.time_from_start = ros::Duration(0.5);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("l_shoulder_pan_joint");
  joint_trajectory.joint_names.push_back("l_shoulder_lift_joint");
  joint_trajectory.joint_names.push_back("l_upper_arm_roll_joint");
  joint_trajectory.joint_names.push_back("l_elbow_flex_joint");
  joint_trajectory.joint_names.push_back("l_forearm_roll_joint");
  joint_trajectory.joint_names.push_back("l_wrist_flex_joint");
  joint_trajectory.joint_names.push_back("l_wrist_roll_joint");
  joint_trajectory.points.push_back(arm_joint_point);

  pub_l_arm_trajectory_.publish(joint_trajectory);
}


void PR2JointState2JointTrajectory::moveRightArm()
{
  std::vector<double> goal_position;
  goal_position.push_back(getCurrentJointStatesAngle("r_shoulder_pan_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("r_shoulder_lift_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("r_upper_arm_roll_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("r_elbow_flex_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("r_forearm_roll_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("r_wrist_flex_joint"));
  goal_position.push_back(getCurrentJointStatesAngle("r_wrist_roll_joint"));

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;
  arm_joint_point.positions = goal_position;
  arm_joint_point.time_from_start = ros::Duration(0.5);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("r_shoulder_pan_joint");
  joint_trajectory.joint_names.push_back("r_shoulder_lift_joint");
  joint_trajectory.joint_names.push_back("r_upper_arm_roll_joint");
  joint_trajectory.joint_names.push_back("r_elbow_flex_joint");
  joint_trajectory.joint_names.push_back("r_forearm_roll_joint");
  joint_trajectory.joint_names.push_back("r_wrist_flex_joint");
  joint_trajectory.joint_names.push_back("r_wrist_roll_joint");
  joint_trajectory.points.push_back(arm_joint_point);

  pub_r_arm_trajectory_.publish(joint_trajectory);
}


void PR2JointState2JointTrajectory::moveLeftHand()
{
  float joint_angle     = getCurrentJointStatesAngle("l_gripper_l_finger_joint");
  float target_position = (joint_angle / 0.55) * 0.086;

  pr2_controllers_msgs::Pr2GripperCommand send_msg;
  send_msg.position = target_position;

  pub_l_hand_command_.publish(send_msg);
}


void PR2JointState2JointTrajectory::moveRightHand()
{
  float joint_angle     = getCurrentJointStatesAngle("r_gripper_l_finger_joint");
  float target_position = (joint_angle / 0.55) * 0.086;

  pr2_controllers_msgs::Pr2GripperCommand send_msg;
  send_msg.position = target_position;

  pub_r_hand_command_.publish(send_msg);
}


int PR2JointState2JointTrajectory::run()
{
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    moveHead();
    moveTorso();
    moveLeftArm();
    moveRightArm();
    moveLeftHand();
    moveRightHand();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_joint_state_2_joint_trajectory");
  PR2JointState2JointTrajectory pr2_joint_state_2_joint_trajectory;
  return pr2_joint_state_2_joint_trajectory.run();
}
