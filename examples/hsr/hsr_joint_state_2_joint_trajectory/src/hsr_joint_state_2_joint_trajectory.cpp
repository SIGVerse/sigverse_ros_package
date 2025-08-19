#include <memory>
#include <signal.h>
#include <functional>
#include <algorithm>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class HsrJointState2JointTrajectory
{
public:

  HsrJointState2JointTrajectory();
  int run();

private:

  static void rosSigintHandler([[maybe_unused]] int sig);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  bool isContainJointNames(std::vector<std::string> joint_names);
  double getCurrentJointStatesAngle(std::string joint_name);
  void moveArm();
  void moveHead();
  void moveGripper();

  sensor_msgs::msg::JointState current_joint_states_;

  std::vector<std::string> arm_joint_names_;
  std::vector<std::string> head_joint_names_;
  std::vector<std::string> gripper_joint_names_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr  pub_arm_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr  pub_head_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr  pub_gripper_trajectory_;
};


HsrJointState2JointTrajectory::HsrJointState2JointTrajectory()
{
  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  arm_joint_names_.push_back("arm_lift_joint");
  arm_joint_names_.push_back("arm_flex_joint");
  arm_joint_names_.push_back("arm_roll_joint");
  arm_joint_names_.push_back("wrist_flex_joint");
  arm_joint_names_.push_back("wrist_roll_joint");
  head_joint_names_.push_back("head_pan_joint");
  head_joint_names_.push_back("head_tilt_joint");
  gripper_joint_names_.push_back("hand_motor_joint");

  node_ = rclcpp::Node::make_shared("hsr_joint_state_2_joint_trajectory");

  sub_joint_state_        = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&HsrJointState2JointTrajectory::jointStateCallback, this, std::placeholders::_1));
  pub_arm_trajectory_     = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
  pub_head_trajectory_    = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/head_trajectory_controller/command", 10);
  pub_gripper_trajectory_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/gripper_controller/command", 10);
}


void HsrJointState2JointTrajectory::rosSigintHandler([[maybe_unused]] int sig)
{
  rclcpp::shutdown();
}


void HsrJointState2JointTrajectory::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
  current_joint_states_ = *joint_state;
}


bool HsrJointState2JointTrajectory::isContainJointNames(std::vector<std::string> joint_names)
{
  if(current_joint_states_.name.empty()){ return false; }

  for(size_t i = 0; i < joint_names.size(); i++)
  {
    std::vector<std::string>::iterator it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(), joint_names[i]);
    if(it == current_joint_states_.name.end()){ return false; }
  }

  return true;
}


double HsrJointState2JointTrajectory::getCurrentJointStatesAngle(std::string joint_name)
{
  std::vector<std::string>::iterator it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(), joint_name);
  int index = std::distance(current_joint_states_.name.begin(), it);
  return current_joint_states_.position[index];
}


void HsrJointState2JointTrajectory::moveArm()
{
  if(isContainJointNames(arm_joint_names_) == false){ return; }

  std::vector<double> goal_position;

  for(size_t i = 0; i < arm_joint_names_.size(); i++){
    goal_position.push_back(getCurrentJointStatesAngle(arm_joint_names_[i]));
  }

  trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;
  arm_joint_point.positions = goal_position;
  arm_joint_point.time_from_start = rclcpp::Duration::from_seconds(0.5);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = arm_joint_names_;
  joint_trajectory.points.push_back(arm_joint_point);

  pub_arm_trajectory_->publish(joint_trajectory);
}


void HsrJointState2JointTrajectory::moveHead()
{
  if(isContainJointNames(head_joint_names_) == false){ return; }

  std::vector<double> goal_position;

  for(size_t i = 0; i < head_joint_names_.size(); i++){
    goal_position.push_back(getCurrentJointStatesAngle(head_joint_names_[i]));
  }

  trajectory_msgs::msg::JointTrajectoryPoint head_joint_point;
  head_joint_point.positions = goal_position;
  head_joint_point.time_from_start = rclcpp::Duration::from_seconds(0.5);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = head_joint_names_;
  joint_trajectory.points.push_back(head_joint_point);

  pub_head_trajectory_->publish(joint_trajectory);
}


void HsrJointState2JointTrajectory::moveGripper()
{
  if(isContainJointNames(gripper_joint_names_) == false){ return; }

  std::vector<double> goal_position;

  for(size_t i = 0; i < gripper_joint_names_.size(); i++){
    goal_position.push_back(getCurrentJointStatesAngle(gripper_joint_names_[i]));
  }

  trajectory_msgs::msg::JointTrajectoryPoint gripper_joint_point;
  gripper_joint_point.positions = goal_position;
  gripper_joint_point.time_from_start = rclcpp::Duration::from_seconds(0.5);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = gripper_joint_names_;
  joint_trajectory.points.push_back(gripper_joint_point);

  pub_gripper_trajectory_->publish(joint_trajectory);
}


int HsrJointState2JointTrajectory::run()
{
  rclcpp::Rate loop_rate(10);

  while (rclcpp::ok())
  {
    moveArm();
    moveHead();
    moveGripper();

    rclcpp::spin_some(node_);
    loop_rate.sleep();
  }

  return 0;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  HsrJointState2JointTrajectory hsr_joint_state_2_joint_trajectory;
  return hsr_joint_state_2_joint_trajectory.run();
}
