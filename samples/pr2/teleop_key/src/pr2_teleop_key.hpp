#ifndef SIGVERSE_PR2_TELEOP_KEY_HPP
#define SIGVERSE_PR2_TELEOP_KEY_HPP

#include <stdio.h>
#include <string>
#include <algorithm>
#include <cmath>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>

class SIGVersePr2TeleopKey
{
private:
//  static const char KEYCODE_1 = 0x31;
//  static const char KEYCODE_2 = 0x32;

  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;

  static const char KEYCODE_D = 0x64;
  static const char KEYCODE_E = 0x65;
  static const char KEYCODE_F = 0x66;
  static const char KEYCODE_G = 0x67;
  static const char KEYCODE_H = 0x68;
  static const char KEYCODE_I = 0x69;
  static const char KEYCODE_J = 0x6a;
  static const char KEYCODE_K = 0x6b;
  static const char KEYCODE_L = 0x6c;
  static const char KEYCODE_O = 0x6f;
  static const char KEYCODE_Q = 0x71;
  static const char KEYCODE_R = 0x72;
  static const char KEYCODE_S = 0x73;
  static const char KEYCODE_T = 0x74;
  static const char KEYCODE_U = 0x75;
  static const char KEYCODE_W = 0x77;
  static const char KEYCODE_Y = 0x79;
  static const char KEYCODE_Z = 0x7a;

  static const char KEYCODE_SPACE  = 0x20;

  enum HandType { Left, Right, };

public:
  SIGVersePr2TeleopKey();

  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  void moveBase(double linear_x, double linear_y, double angular_z);
  void operateHead (const std::string &joint_name, const double add_pos);
  void operateTorso(const std::string &joint_name, const double add_pos);
  void operateArm  (const HandType hand_type, const std::string &joint_type, const double add_pos);
  void operateGripper(const HandType hand_type, const float is_open);

  void showHelp();
  void showHelpHead();
  void showHelpTorso();
  void showHelpArm(const std::string &arm_type);

  double getDurationPos(const double next_pos, const double current_pos);
  double getDurationRot(const double next_pos, const double current_pos);

  int run();

private:

  std::map<std::string, double> joint_state_map_;
  std::map<std::string, double> joint_max_map_;
  std::map<std::string, double> joint_min_map_;

  ros::NodeHandle node_handle_;

  ros::Subscriber sub_joint_state_;
  ros::Publisher  pub_base_twist_;
  ros::Publisher  pub_head_trajectory_;
  ros::Publisher  pub_torso_trajectory_;
  ros::Publisher  pub_l_arm_trajectory_;
  ros::Publisher  pub_r_arm_trajectory_;
  ros::Publisher  pub_l_gripper_command_;
  ros::Publisher  pub_r_gripper_command_;

  tf::TransformListener listener_;
};


SIGVersePr2TeleopKey::SIGVersePr2TeleopKey()
{
  joint_state_map_["head_pan_joint"]         = 0.0;
  joint_state_map_["head_tilt_joint"]        = 0.0;
  joint_state_map_["torso_lift_joint"]       = 0.0;
  joint_state_map_["l_shoulder_pan_joint"]   = 0.0;
  joint_state_map_["l_shoulder_lift_joint"]  = 0.0;
  joint_state_map_["l_upper_arm_roll_joint"] = 0.0;
  joint_state_map_["l_elbow_flex_joint"]     = 0.0;
  joint_state_map_["l_forearm_roll_joint"]   = 0.0;
  joint_state_map_["l_wrist_flex_joint"]     = 0.0;
  joint_state_map_["l_wrist_roll_joint"]     = 0.0;
  joint_state_map_["r_shoulder_pan_joint"]   = 0.0;
  joint_state_map_["r_shoulder_lift_joint"]  = 0.0;
  joint_state_map_["r_upper_arm_roll_joint"] = 0.0;
  joint_state_map_["r_elbow_flex_joint"]     = 0.0;
  joint_state_map_["r_forearm_roll_joint"]   = 0.0;
  joint_state_map_["r_wrist_flex_joint"]     = 0.0;
  joint_state_map_["r_wrist_roll_joint"]     = 0.0;

  joint_max_map_["head_pan_joint"]         = +2.93;
  joint_max_map_["head_tilt_joint"]        = +1.04;
  joint_max_map_["torso_lift_joint"]       = +0.31;

  joint_max_map_["l_shoulder_pan_joint"]   = +2.26;
  joint_max_map_["l_shoulder_lift_joint"]  = +1.39;
  joint_max_map_["l_upper_arm_roll_joint"] = +3.90;
  joint_max_map_["l_elbow_flex_joint"]     = +0.0;
  joint_max_map_["l_forearm_roll_joint"]   = +9.99;
  joint_max_map_["l_wrist_flex_joint"]     = +2.26;
  joint_max_map_["l_wrist_roll_joint"]     = +9.99;

  joint_max_map_["r_shoulder_pan_joint"]   = +0.69;
  joint_max_map_["r_shoulder_lift_joint"]  = +1.39;
  joint_max_map_["r_upper_arm_roll_joint"] = +0.76;
  joint_max_map_["r_elbow_flex_joint"]     = +0.0;
  joint_max_map_["r_forearm_roll_joint"]   = +9.99;
  joint_max_map_["r_wrist_flex_joint"]     = +0.0;
  joint_max_map_["r_wrist_roll_joint"]     = +9.99;

  joint_min_map_["head_pan_joint"]         = -2.93;
  joint_min_map_["head_tilt_joint"]        = -0.52;
  joint_min_map_["torso_lift_joint"]       = +0.0;

  joint_min_map_["l_shoulder_pan_joint"]   = -0.69;
  joint_min_map_["l_shoulder_lift_joint"]  = -0.52;
  joint_min_map_["l_upper_arm_roll_joint"] = -0.76;
  joint_min_map_["l_elbow_flex_joint"]     = -2.32;
  joint_min_map_["l_forearm_roll_joint"]   = -9.99;
  joint_min_map_["l_wrist_flex_joint"]     = -2.26;
  joint_min_map_["l_wrist_roll_joint"]     = -9.99;

  joint_min_map_["r_shoulder_pan_joint"]   = -2.26;
  joint_min_map_["r_shoulder_lift_joint"]  = -0.52;
  joint_min_map_["r_upper_arm_roll_joint"] = -3.90;
  joint_min_map_["r_elbow_flex_joint"]     = -2.32;
  joint_min_map_["r_forearm_roll_joint"]   = -9.99;
  joint_min_map_["r_wrist_flex_joint"]     = -2.26;
  joint_min_map_["r_wrist_roll_joint"]     = -9.99;
}


void SIGVersePr2TeleopKey::showHelp()
{
  puts("");
  puts("---------------------------");
  puts("Operate by Keyboard");
  puts("---------------------------");
  puts("arrow keys : Move HSR");
  puts("space      : Stop HSR");
  puts("q/z : Increase/Decrease Moving Speed");
  puts("---------------------------");
  puts("h : Operate Head");
  puts("---------------------------");
  puts("t : Operate Torso");
  puts("---------------------------");
  puts("l/r : Operate Left/Right Arm");
  puts("");
}

void SIGVersePr2TeleopKey::showHelpHead()
{
  puts("");
  puts("---------------------------");
  puts("Operate Head");
  puts("---------------------------");
  puts("r/f : Left/Right Head pan");
  puts("t/g : Up/Down    Head tilt");
  puts("---------------------------");
  puts("space : Stop");
  puts("---------------------------");
  puts("q : Quit");
  puts("");
}

void SIGVersePr2TeleopKey::showHelpTorso()
{
  puts("");
  puts("---------------------------");
  puts("Operate Torso");
  puts("---------------------------");
  puts("r/f : Up/Down Torso lift");
  puts("---------------------------");
  puts("space : Stop");
  puts("---------------------------");
  puts("q : Quit");
  puts("");
}

void SIGVersePr2TeleopKey::showHelpArm(const std::string &arm_type)
{
  puts("");
  puts("---------------------------");
  puts(("Operate " + arm_type).c_str());
  puts("---------------------------");
  puts("w/s : +/- Shoulder pan");
  puts("e/d : +/- Shoulder lift");
  puts("r/f : +/- Upper arm roll");
  puts("t/g : +/- Elbow flex");
  puts("y/h : +/- Forearm roll");
  puts("u/j : +/- Wrist flex");
  puts("i/k : +/- Wrist roll");
  puts("o/l : Open/Close Gripper");
  puts("---------------------------");
  puts("space : Stop");
  puts("---------------------------");
  puts("q : Quit");
  puts("");
}


#endif // SIGVERSE_PR2_TELEOP_KEY_HPP

