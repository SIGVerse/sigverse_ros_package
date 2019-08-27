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
  bool isContainJointNames(std::vector<std::string> joint_names);
  double getCurrentJointStatesAngle(std::string joint_name);
  void moveHead();
  void moveTorso();
  void moveLeftArm();
  void moveRightArm();
  void moveLeftHand();
  void moveRightHand();

  sensor_msgs::JointState current_joint_states_;

  std::vector<std::string> head_joint_names_;
  std::vector<std::string> torso_joint_names_;
  std::vector<std::string> l_arm_joint_names_;
  std::vector<std::string> r_arm_joint_names_;
  std::vector<std::string> l_hand_joint_names_;
  std::vector<std::string> r_hand_joint_names_;

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

  head_joint_names_.push_back("head_pan_joint");
  head_joint_names_.push_back("head_tilt_joint");
  torso_joint_names_.push_back("torso_lift_joint");
  l_arm_joint_names_.push_back("l_shoulder_pan_joint");
  l_arm_joint_names_.push_back("l_shoulder_lift_joint");
  l_arm_joint_names_.push_back("l_upper_arm_roll_joint");
  l_arm_joint_names_.push_back("l_elbow_flex_joint");
  l_arm_joint_names_.push_back("l_forearm_roll_joint");
  l_arm_joint_names_.push_back("l_wrist_flex_joint");
  l_arm_joint_names_.push_back("l_wrist_roll_joint");
  r_arm_joint_names_.push_back("r_shoulder_pan_joint");
  r_arm_joint_names_.push_back("r_shoulder_lift_joint");
  r_arm_joint_names_.push_back("r_upper_arm_roll_joint");
  r_arm_joint_names_.push_back("r_elbow_flex_joint");
  r_arm_joint_names_.push_back("r_forearm_roll_joint");
  r_arm_joint_names_.push_back("r_wrist_flex_joint");
  r_arm_joint_names_.push_back("r_wrist_roll_joint");
  l_hand_joint_names_.push_back("l_gripper_l_finger_joint");
  r_hand_joint_names_.push_back("r_gripper_l_finger_joint");

  std::string sub_joint_state_topic_name;
  std::string pub_head_trajectory_topic_name;
  std::string pub_torso_trajectory_topic_name;
  std::string pub_l_arm_trajectory_topic_name;
  std::string pub_r_arm_trajectory_topic_name;
  std::string pub_l_hand_topic_name;
  std::string pub_r_hand_topic_name;

  node_handle_.param<std::string>("pr2_joint_state_2_joint_trajectory/sub_joint_state_topic_name",        sub_joint_state_topic_name,        "/joint_states");
  node_handle_.param<std::string>("pr2_joint_state_2_joint_trajectory/pub_head_trajectory_topic_name",    pub_head_trajectory_topic_name,    "/head_traj_controller/command");
  node_handle_.param<std::string>("pr2_joint_state_2_joint_trajectory/pub_torso_trajectory_topic_name",   pub_torso_trajectory_topic_name,   "/torso_controller/command");
  node_handle_.param<std::string>("pr2_joint_state_2_joint_trajectory/pub_l_arm_trajectory_topic_name",   pub_l_arm_trajectory_topic_name,   "/l_arm_controller/command");
  node_handle_.param<std::string>("pr2_joint_state_2_joint_trajectory/pub_r_arm_trajectory_topic_name",   pub_r_arm_trajectory_topic_name,   "/r_arm_controller/command");
  node_handle_.param<std::string>("pr2_joint_state_2_joint_trajectory/pub_l_hand_topic_name",             pub_l_hand_topic_name,             "/l_gripper_controller/command");
  node_handle_.param<std::string>("pr2_joint_state_2_joint_trajectory/pub_r_hand_topic_name",             pub_r_hand_topic_name,             "/r_gripper_controller/command");

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


bool PR2JointState2JointTrajectory::isContainJointNames(std::vector<std::string> joint_names)
{
  if(current_joint_states_.name.empty()){ return false; }

  for(int i = 0; i < joint_names.size(); i++)
  {
    std::vector<std::string>::iterator it =std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(), joint_names[i]);
    if(it == current_joint_states_.name.end()){ return false; }
  }

  return true;
}


double PR2JointState2JointTrajectory::getCurrentJointStatesAngle(std::string joint_name)
{
    std::vector<std::string>::iterator it =std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(), joint_name);
    int index = std::distance(current_joint_states_.name.begin(), it);
    return current_joint_states_.position[index];
}


void PR2JointState2JointTrajectory::moveHead()
{
  if(isContainJointNames(head_joint_names_) == false){ return; }

  std::vector<double> goal_position;

  for(int i = 0; i < head_joint_names_.size(); i++){
    goal_position.push_back(getCurrentJointStatesAngle(head_joint_names_[i]));
  }

  trajectory_msgs::JointTrajectoryPoint head_joint_point;
  head_joint_point.positions = goal_position;
  head_joint_point.time_from_start = ros::Duration(0.5);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = head_joint_names_;
  joint_trajectory.points.push_back(head_joint_point);

  pub_head_trajectory_.publish(joint_trajectory);
}


void PR2JointState2JointTrajectory::moveTorso()
{
  if(isContainJointNames(torso_joint_names_) == false){ return; }

  std::vector<double> goal_position;

  for(int i = 0; i < torso_joint_names_.size(); i++){
    goal_position.push_back(getCurrentJointStatesAngle(torso_joint_names_[i]));
  }

  trajectory_msgs::JointTrajectoryPoint gripper_joint_point;
  gripper_joint_point.positions = goal_position;
  gripper_joint_point.time_from_start = ros::Duration(0.5);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = torso_joint_names_;
  joint_trajectory.points.push_back(gripper_joint_point);

  pub_torso_trajectory_.publish(joint_trajectory);
}


void PR2JointState2JointTrajectory::moveLeftArm()
{
  if(isContainJointNames(l_arm_joint_names_) == false){ return; }

  std::vector<double> goal_position;

  for(int i = 0; i < l_arm_joint_names_.size(); i++){
    goal_position.push_back(getCurrentJointStatesAngle(l_arm_joint_names_[i]));
  }

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;
  arm_joint_point.positions = goal_position;
  arm_joint_point.time_from_start = ros::Duration(0.5);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = l_arm_joint_names_;
  joint_trajectory.points.push_back(arm_joint_point);

  pub_l_arm_trajectory_.publish(joint_trajectory);
}


void PR2JointState2JointTrajectory::moveRightArm()
{
  if(isContainJointNames(r_arm_joint_names_) == false){ return; }

  std::vector<double> goal_position;

  for(int i = 0; i < r_arm_joint_names_.size(); i++){
    goal_position.push_back(getCurrentJointStatesAngle(r_arm_joint_names_[i]));
  }

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;
  arm_joint_point.positions = goal_position;
  arm_joint_point.time_from_start = ros::Duration(0.5);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = r_arm_joint_names_;
  joint_trajectory.points.push_back(arm_joint_point);

  pub_r_arm_trajectory_.publish(joint_trajectory);
}


void PR2JointState2JointTrajectory::moveLeftHand()
{
  if(isContainJointNames(l_hand_joint_names_) == false){ return; }

  float joint_angle     = getCurrentJointStatesAngle(l_hand_joint_names_[0]);
  float target_position = (joint_angle / 0.55) * 0.086;

  pr2_controllers_msgs::Pr2GripperCommand send_msg;
  send_msg.position = target_position;

  pub_l_hand_command_.publish(send_msg);
}


void PR2JointState2JointTrajectory::moveRightHand()
{
  if(isContainJointNames(r_hand_joint_names_) == false){ return; }

  float joint_angle     = getCurrentJointStatesAngle(r_hand_joint_names_[0]);
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
