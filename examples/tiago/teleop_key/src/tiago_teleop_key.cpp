#include <memory>
#include <cmath>
#include <signal.h>
#include <termios.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class SIGVerseTiagoTeleopKey
{
private:

  static const char KEYCODE_1 = 0x31;
  static const char KEYCODE_2 = 0x32;
  static const char KEYCODE_3 = 0x33;
  static const char KEYCODE_4 = 0x34;
  static const char KEYCODE_5 = 0x35;
  static const char KEYCODE_6 = 0x36;
  static const char KEYCODE_7 = 0x37;

  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;

  static const char KEYCODE_A = 0x61;
  static const char KEYCODE_C = 0x63;
  static const char KEYCODE_D = 0x64;
  static const char KEYCODE_E = 0x65;
  static const char KEYCODE_F = 0x66;
  static const char KEYCODE_G = 0x67;
  static const char KEYCODE_H = 0x68;
  static const char KEYCODE_I = 0x69;
  static const char KEYCODE_J = 0x6a;
  static const char KEYCODE_K = 0x6b;
  static const char KEYCODE_L = 0x6c;
  static const char KEYCODE_M = 0x6d;
  static const char KEYCODE_N = 0x6e;
  static const char KEYCODE_O = 0x6f;
  static const char KEYCODE_Q = 0x71;
  static const char KEYCODE_R = 0x72;
  static const char KEYCODE_S = 0x73;
  static const char KEYCODE_U = 0x75;
  static const char KEYCODE_W = 0x77;
  static const char KEYCODE_X = 0x78;
  static const char KEYCODE_Y = 0x79;
  static const char KEYCODE_Z = 0x7a;

  static const char KEYCODE_COMMA  = 0x2c;
  static const char KEYCODE_PERIOD = 0x2e;
  static const char KEYCODE_SPACE  = 0x20;

  const std::string MSG_TELL_ME  = "Please tell me";
  const std::string MSG_POINT_IT = "Please point it";

  std::map<std::string, double> arm_joint_state_map_;
  std::map<std::string, double> arm_joint_max_map_;
  std::map<std::string, double> arm_joint_min_map_;

public:
  SIGVerseTiagoTeleopKey();

  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);

  void messageCallback(const std_msgs::msg::String::SharedPtr message);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  void sendMessage(const std::string &message);
  void moveBaseTwist(double linear_x, double linear_y, double angular_z);
  void operateTorso(const double torso_lift_pos, const double duration_sec);
  void operateHead(const double head_1_pos, const double head_2_pos, const double duration_sec);
  void operateArm(const std::vector<double> & positions, const double duration_sec);
  void operateArm(const int joint_number, const double arm_pos, const double duration_sec);
  void operateHand(bool grasp);

  void showHelp();
  void showHelpArm(const std::string &arm_name);
  int run();

private:
  // Last position and previous position of arm_lift_joint
  double torso_lift_joint_pos1_;
  double torso_lift_joint_pos2_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_torso_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_trajectory_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
};


SIGVerseTiagoTeleopKey::SIGVerseTiagoTeleopKey()
{
  torso_lift_joint_pos1_ = 0.0;
  torso_lift_joint_pos2_ = 0.0;

  arm_joint_state_map_["arm_1_joint"] = 0.0;
  arm_joint_state_map_["arm_2_joint"] = 0.0;
  arm_joint_state_map_["arm_3_joint"] = 0.0;
  arm_joint_state_map_["arm_4_joint"] = 0.0;
  arm_joint_state_map_["arm_5_joint"] = 0.0;
  arm_joint_state_map_["arm_6_joint"] = 0.0;
  arm_joint_state_map_["arm_7_joint"] = 0.0;

  arm_joint_max_map_["arm_1_joint"] = +2.68;
  arm_joint_max_map_["arm_2_joint"] = +1.02;
  arm_joint_max_map_["arm_3_joint"] = +1.50;
  arm_joint_max_map_["arm_4_joint"] = +2.29;
  arm_joint_max_map_["arm_5_joint"] = +2.07;
  arm_joint_max_map_["arm_6_joint"] = +1.39;
  arm_joint_max_map_["arm_7_joint"] = +2.07;

  arm_joint_min_map_["arm_1_joint"] = +0.07;
  arm_joint_min_map_["arm_2_joint"] = -1.50;
  arm_joint_min_map_["arm_3_joint"] = -3.46;
  arm_joint_min_map_["arm_4_joint"] = -0.32;
  arm_joint_min_map_["arm_5_joint"] = -2.07;
  arm_joint_min_map_["arm_6_joint"] = -1.39;
  arm_joint_min_map_["arm_7_joint"] = -2.07;

  node_ = rclcpp::Node::make_shared("tiago_teleop_key");

  pub_msg_                = node_->create_publisher<std_msgs::msg::String>("/tiago/message/to_human", 10);
  pub_base_twist_         = node_->create_publisher<geometry_msgs::msg::Twist>("/mobile_base_controller/cmd_vel", 10);
  pub_torso_trajectory_   = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/torso_controller/command", 10);
  pub_head_trajectory_    = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/head_controller/command", 10);
  pub_arm_trajectory_     = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/command", 10);
  pub_gripper_trajectory_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/gripper_controller/command", 10);
  sub_msg_                = node_->create_subscription<std_msgs::msg::String>("/tiago/message/to_robot", 10, std::bind(&SIGVerseTiagoTeleopKey::messageCallback, this, std::placeholders::_1));
  sub_joint_state_        = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",    10, std::bind(&SIGVerseTiagoTeleopKey::jointStateCallback, this, std::placeholders::_1));
}


void SIGVerseTiagoTeleopKey::rosSigintHandler([[maybe_unused]] int sig)
{
  rclcpp::shutdown();
}


int SIGVerseTiagoTeleopKey::canReceive( int fd )
{
  fd_set fdset;
  struct timeval timeout;
  FD_ZERO( &fdset );
  FD_SET( fd , &fdset );

  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  return select( fd+1 , &fdset , NULL , NULL , &timeout );
}

void SIGVerseTiagoTeleopKey::messageCallback(const std_msgs::msg::String::SharedPtr message)
{
  RCLCPP_INFO(node_->get_logger(), "Subscribe message: %s", message->data.c_str());
}

void SIGVerseTiagoTeleopKey::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
  for(size_t i=0; i<joint_state->name.size(); i++)
  {
    if(joint_state->name[i] == "torso_lift_joint")
    {
      torso_lift_joint_pos2_ = torso_lift_joint_pos1_;
      torso_lift_joint_pos1_ = joint_state->position[i];
    }
    if(joint_state->name[i] == "arm_1_joint"){ arm_joint_state_map_["arm_1_joint"] = joint_state->position[i]; }
    if(joint_state->name[i] == "arm_2_joint"){ arm_joint_state_map_["arm_2_joint"] = joint_state->position[i]; }
    if(joint_state->name[i] == "arm_3_joint"){ arm_joint_state_map_["arm_3_joint"] = joint_state->position[i]; }
    if(joint_state->name[i] == "arm_4_joint"){ arm_joint_state_map_["arm_4_joint"] = joint_state->position[i]; }
    if(joint_state->name[i] == "arm_5_joint"){ arm_joint_state_map_["arm_5_joint"] = joint_state->position[i]; }
    if(joint_state->name[i] == "arm_6_joint"){ arm_joint_state_map_["arm_6_joint"] = joint_state->position[i]; }
    if(joint_state->name[i] == "arm_7_joint"){ arm_joint_state_map_["arm_7_joint"] = joint_state->position[i]; }
  }
}

void SIGVerseTiagoTeleopKey::sendMessage(const std::string &message)
{
  RCLCPP_INFO(node_->get_logger(), "Send message:%s", message.c_str());

  std_msgs::msg::String string_msg;
  string_msg.data = message;
  pub_msg_->publish(string_msg);
}

void SIGVerseTiagoTeleopKey::moveBaseTwist(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::msg::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_->publish(twist);
}

void SIGVerseTiagoTeleopKey::operateTorso(const double torso_lift_pos, const double duration_sec)
{
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("torso_lift_joint");

  trajectory_msgs::msg::JointTrajectoryPoint torso_joint_point;

  torso_joint_point.positions = { torso_lift_pos };

  torso_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  joint_trajectory.points.push_back(torso_joint_point);
  pub_torso_trajectory_->publish(joint_trajectory);
}

void SIGVerseTiagoTeleopKey::operateHead(const double head_1_pos, const double head_2_pos, const double duration_sec)
{
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("head_1_joint");
  joint_trajectory.joint_names.push_back("head_2_joint");

  trajectory_msgs::msg::JointTrajectoryPoint head_joint_point;

  head_joint_point.positions = { head_1_pos, head_2_pos };

  head_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  joint_trajectory.points.push_back(head_joint_point);
  pub_torso_trajectory_->publish(joint_trajectory);
}

void SIGVerseTiagoTeleopKey::operateArm(const std::vector<double> & positions, const double duration_sec)
{
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("arm_1_joint");
  joint_trajectory.joint_names.push_back("arm_2_joint");
  joint_trajectory.joint_names.push_back("arm_3_joint");
  joint_trajectory.joint_names.push_back("arm_4_joint");
  joint_trajectory.joint_names.push_back("arm_5_joint");
  joint_trajectory.joint_names.push_back("arm_6_joint");
  joint_trajectory.joint_names.push_back("arm_7_joint");

  trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

  arm_joint_point.positions = positions;

  arm_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  joint_trajectory.points.push_back(arm_joint_point);
  pub_arm_trajectory_->publish(joint_trajectory);
}

void SIGVerseTiagoTeleopKey::operateArm(const int joint_number, const double add_pos, const double duration_sec)
{
  std::string joint_name = "arm_" + std::to_string(joint_number) + "_joint";

  std::vector<double> positions =
  {
    arm_joint_state_map_["arm_1_joint"], arm_joint_state_map_["arm_2_joint"], arm_joint_state_map_["arm_3_joint"],
    arm_joint_state_map_["arm_4_joint"], arm_joint_state_map_["arm_5_joint"], arm_joint_state_map_["arm_6_joint"],
    arm_joint_state_map_["arm_7_joint"]
  };

  // Clamp
  double clamped_next_pos = std::min(std::max(arm_joint_min_map_[joint_name], arm_joint_state_map_[joint_name]+add_pos), arm_joint_max_map_[joint_name]);

  positions[joint_number-1] = clamped_next_pos;

  operateArm(positions, duration_sec);
}

//double SIGVerseTiagoTeleopKey::getDurationRot(const double next_pos, const double current_pos)
//{
//  return std::max<double>((std::abs(next_pos - current_pos) * 1.2), 1.0);
//}

void SIGVerseTiagoTeleopKey::operateHand(bool is_hand_open)
{
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("gripper_left_finger_joint");
  joint_trajectory.joint_names.push_back("gripper_right_finger_joint");

  trajectory_msgs::msg::JointTrajectoryPoint gripper_joint_point;

  if(is_hand_open)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Grasp");
    gripper_joint_point.positions = { 0.0, 0.0 };
  }
  else
  {
    RCLCPP_DEBUG(node_->get_logger(), "Open hand");
    gripper_joint_point.positions = { 0.04, 0.04 };
  }

  gripper_joint_point.time_from_start = rclcpp::Duration::from_seconds(2);
  joint_trajectory.points.push_back(gripper_joint_point);
  pub_gripper_trajectory_->publish(joint_trajectory);
}


void SIGVerseTiagoTeleopKey::showHelp()
{
  puts("Operate by Keyboard");
  puts("---------------------------");
  puts("arrow keys : Move TIAGo");
  puts("space      : Stop TIAGo");
  puts("r/f : Increase/Decrease Moving Speed");
  puts("---------------------------");
  puts("y : Rotate Arm - Upward");
  puts("h : Rotate Arm - Horizontal");
  puts("n : Rotate Arm - Downward");
  puts("---------------------------");
  puts("q/a/z : Up/Stop/Down Torso");
  puts("---------------------------");
  puts("w/s/x : Turn Head Left/Front/Right ");
  puts("e/d/c : Turn Head Up/Front/Down");
  puts("---------------------------");
  puts("u/j/m : Control Arm1/Arm2/Arm3 ");
  puts("i/k   : Control Arm4/Arm5 ");
  puts("o/l   : Control Arm6/Arm7 ");
  puts("---------------------------");
  puts("g : Grasp/Open Hand");
  puts("---------------------------");
  puts(("1 : Send Message: "+MSG_TELL_ME).c_str());
  puts(("2 : Send Message: "+MSG_POINT_IT).c_str());
}

void SIGVerseTiagoTeleopKey::showHelpArm(const std::string &arm_name)
{
  puts("");
  puts("---------------------------");
  puts(("Control "+arm_name).c_str());
  puts("u/j/m : + / Stop / - ");
  puts("---------------------------");
  puts("q : Quit");
  puts("");
}


int SIGVerseTiagoTeleopKey::run()
{
  char c;
  int  ret;
  char buf[1024];

  /////////////////////////////////////////////
  // get the console in raw mode
  int kfd = 0;
  struct termios cooked;

  struct termios raw;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  /////////////////////////////////////////////

  showHelp();

  // Override the default ros sigint handler.
  // This must be set after the first Node is created.
  signal(SIGINT, rosSigintHandler);

  auto logger = node_->get_logger();

  rclcpp::Rate loop_rate(40);

  const float linear_coef  = 0.2f;
  const float angular_coef = 0.5f;

  float move_speed = 1.0f;
  bool is_hand_open = false;

  std::string torso_lift_joint_name = "arm_lift_joint";
  std::string arm_flex_joint_name   = "arm_flex_joint";
  std::string wrist_flex_joint_name = "wrist_flex_joint";

  while (rclcpp::ok())
  {
    if(canReceive(kfd))
    {
      // get the next event from the keyboard
      if((ret = read(kfd, &buf, sizeof(buf))) < 0)
      {
        perror("read():");
        exit(EXIT_FAILURE);
      }

      c = buf[ret-1];
          
      switch(c)
      {
        case KEYCODE_1:
        {
          sendMessage(MSG_TELL_ME);
          break;
        }
        case KEYCODE_2:
        {
          sendMessage(MSG_POINT_IT);
          break;
        }
        case KEYCODE_UP:
        {
          RCLCPP_DEBUG(logger, "Go Forward");
          moveBaseTwist(+linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_DOWN:
        {
          RCLCPP_DEBUG(logger, "Go Backward");
          moveBaseTwist(-linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_RIGHT:
        {
          RCLCPP_DEBUG(logger, "Go Right");
          moveBaseTwist(0.0, 0.0, -angular_coef*move_speed);
          break;
        }
        case KEYCODE_LEFT:
        {
          RCLCPP_DEBUG(logger, "Go Left");
          moveBaseTwist(0.0, 0.0, +angular_coef*move_speed);
          break;
        }
        case KEYCODE_SPACE:
        {
          RCLCPP_DEBUG(logger, "Stop");
          moveBaseTwist(0.0, 0.0, 0.0);
          break;
        }
        case KEYCODE_R:
        {
          RCLCPP_DEBUG(logger, "Move Speed Up");
          move_speed *= 2;
          if(move_speed > 2  ){ move_speed=2; }
          break;
        }
        case KEYCODE_F:
        {
          RCLCPP_DEBUG(logger, "Move Speed Down");
          move_speed /= 2;
          if(move_speed < 0.125){ move_speed=0.125; }
          break;
        }
        case KEYCODE_Y:
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Upward");
          operateArm({ 1.57, -1.4, -3.14, 2.2, 1.57, 0.0, 0.0 }, 3.0);
          break;
        }
        case KEYCODE_H:
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Horizontal");
          operateArm({ 1.57, -1.4, -3.14, 2.0, 1.57, 0.6, 0.0 }, 3.0);
          break;
        }
        case KEYCODE_N:
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Downward");
          operateArm({ 1.57, -1.3, -3.14, 0.7, 1.57, 0.0, 0.0 }, 3.0);
          break;
        }
        case KEYCODE_Q:
        {
          RCLCPP_DEBUG(logger, "Torso Height - Up");
          operateTorso(0.35, std::max<int>((int)(std::abs(0.35 - torso_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case KEYCODE_A:
        {
          RCLCPP_DEBUG(logger, "Torso Height - Stop");
          operateTorso(2.0*torso_lift_joint_pos1_-torso_lift_joint_pos2_, 0.5);
          break;
        }
        case KEYCODE_Z:
        {
          RCLCPP_DEBUG(logger, "Torso Height - Down");
          operateTorso(0.0, std::max<int>((int)(std::abs(0.0 - torso_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case KEYCODE_W:
        {
          RCLCPP_DEBUG(logger, "Turn Head - Left");
          operateHead(+1.24, 0.0, 2.0);
          break;
        }
        case KEYCODE_S:
        {
          RCLCPP_DEBUG(logger, "Turn Head - Front");
          operateHead(0.0, 0.0, 2.0);
          break;
        }
        case KEYCODE_X:
        {
          RCLCPP_DEBUG(logger, "Turn Head - Right");
          operateHead(-1.24, 0.0, 2.0);
          break;
        }
        case KEYCODE_E:
        {
          RCLCPP_DEBUG(logger, "Turn Head - Up");
          operateHead(0.0, 0.79, 2.0);
          break;
        }
        case KEYCODE_D:
        {
          RCLCPP_DEBUG(logger, "Turn Head - Front");
          operateHead(0.0, 0.0, 2.0);
          break;
        }
        case KEYCODE_C:
        {
          RCLCPP_DEBUG(logger, "Turn Head - Down");
          operateHead(0.0, -0.98, 2.0);
          break;
        }
        case KEYCODE_U:
        case KEYCODE_J:
        case KEYCODE_M:
        case KEYCODE_I:
        case KEYCODE_K:
        case KEYCODE_O:
        case KEYCODE_L:
        {
          int joint_number;

          switch(c)
          {
            case KEYCODE_U:{ joint_number = 1; RCLCPP_DEBUG(logger, "Control Arm1"); showHelpArm("Arm1"); break; }
            case KEYCODE_J:{ joint_number = 2; RCLCPP_DEBUG(logger, "Control Arm2"); showHelpArm("Arm2"); break; }
            case KEYCODE_M:{ joint_number = 3; RCLCPP_DEBUG(logger, "Control Arm3"); showHelpArm("Arm3"); break; }
            case KEYCODE_I:{ joint_number = 4; RCLCPP_DEBUG(logger, "Control Arm4"); showHelpArm("Arm4"); break; }
            case KEYCODE_K:{ joint_number = 5; RCLCPP_DEBUG(logger, "Control Arm5"); showHelpArm("Arm5"); break; }
            case KEYCODE_O:{ joint_number = 6; RCLCPP_DEBUG(logger, "Control Arm6"); showHelpArm("Arm6"); break; }
            case KEYCODE_L:{ joint_number = 7; RCLCPP_DEBUG(logger, "Control Arm7"); showHelpArm("Arm7"); break; }
          }

          bool is_in_control = true;

          while(is_in_control)
          {
            rclcpp::spin_some(node_);
            loop_rate.sleep();

            if(!canReceive(kfd)){ continue; }

            if(read(kfd, &c, 1) < 0)
            {
              perror("read():");
              exit(EXIT_FAILURE);
            }

            switch(c)
            {
              case KEYCODE_U:{ RCLCPP_DEBUG(logger, "Arm  +  "); operateArm(joint_number, +1.0, 2.0); break; }
              case KEYCODE_J:{ RCLCPP_DEBUG(logger, "Arm Stop"); operateArm(joint_number,  0.0, 2.0); break; }
              case KEYCODE_M:{ RCLCPP_DEBUG(logger, "Arm  -  "); operateArm(joint_number, -1.0, 2.0); break; }
              case KEYCODE_Q:
              {
                is_in_control = false;
                showHelp();
                break;
              }
            }
          }
          break;
        }
        case KEYCODE_G:
        {
          operateHand(is_hand_open);

          is_hand_open = !is_hand_open;
          break;
        }
      }
    }

    rclcpp::spin_some(node_);

    loop_rate.sleep();
  }

  /////////////////////////////////////////////
  // cooked mode
  tcsetattr(kfd, TCSANOW, &cooked);
  /////////////////////////////////////////////

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}


int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
  rclcpp::init(argc, argv);
  SIGVerseTiagoTeleopKey tiago_teleop_key;
  return tiago_teleop_key.run();
}