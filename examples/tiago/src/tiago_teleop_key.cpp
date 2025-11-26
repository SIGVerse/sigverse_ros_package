#include <memory>
#include <cmath>
#include <termios.h>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class SIGVerseTiagoTeleopKey
{
private:
  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;

  const std::string MSG_TELL_ME  = "Please tell me";
  const std::string MSG_POINT_IT = "Please point it";

  std::map<std::string, double> arm_joint_state_map_;
  std::map<std::string, double> arm_joint_max_map_;
  std::map<std::string, double> arm_joint_min_map_;

public:
  SIGVerseTiagoTeleopKey();

  static int  can_receive(int fd);

  void message_callback(const std_msgs::msg::String::SharedPtr message);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  void send_message(const std::string &message);
  void move_base_twist(double linear_x, double linear_y, double angular_z);
  void operate_torso(const double torso_lift_pos, const double duration_sec);
  void operate_head(const double head_1_pos, const double head_2_pos, const double duration_sec);
  void operate_arm(const std::vector<double> & positions, const double duration_sec);
  void operate_arm(const int joint_number, const double arm_pos, const double duration_sec);
  void operate_hand(bool grasp);

  void show_help();
  void show_help_arm(const std::string &arm_name);
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
  sub_msg_                = node_->create_subscription<std_msgs::msg::String>("/tiago/message/to_robot", 10, std::bind(&SIGVerseTiagoTeleopKey::message_callback, this, std::placeholders::_1));
  sub_joint_state_        = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",    10, std::bind(&SIGVerseTiagoTeleopKey::joint_state_callback, this, std::placeholders::_1));
}


int SIGVerseTiagoTeleopKey::can_receive( int fd )
{
  fd_set fdset;
  struct timeval timeout;
  FD_ZERO( &fdset );
  FD_SET( fd , &fdset );

  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  return select( fd+1 , &fdset , NULL , NULL , &timeout );
}

void SIGVerseTiagoTeleopKey::message_callback(const std_msgs::msg::String::SharedPtr message)
{
  RCLCPP_INFO(node_->get_logger(), "Subscribe message: %s", message->data.c_str());
}

void SIGVerseTiagoTeleopKey::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
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

void SIGVerseTiagoTeleopKey::send_message(const std::string &message)
{
  RCLCPP_INFO(node_->get_logger(), "Send message:%s", message.c_str());

  std_msgs::msg::String string_msg;
  string_msg.data = message;
  pub_msg_->publish(string_msg);
}

void SIGVerseTiagoTeleopKey::move_base_twist(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::msg::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_->publish(twist);
}

void SIGVerseTiagoTeleopKey::operate_torso(const double torso_lift_pos, const double duration_sec)
{
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("torso_lift_joint");

  trajectory_msgs::msg::JointTrajectoryPoint torso_joint_point;

  torso_joint_point.positions = { torso_lift_pos };

  torso_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  joint_trajectory.points.push_back(torso_joint_point);
  pub_torso_trajectory_->publish(joint_trajectory);
}

void SIGVerseTiagoTeleopKey::operate_head(const double head_1_pos, const double head_2_pos, const double duration_sec)
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

void SIGVerseTiagoTeleopKey::operate_arm(const std::vector<double> & positions, const double duration_sec)
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

void SIGVerseTiagoTeleopKey::operate_arm(const int joint_number, const double add_pos, const double duration_sec)
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

  operate_arm(positions, duration_sec);
}

//double SIGVerseTiagoTeleopKey::getDurationRot(const double next_pos, const double current_pos)
//{
//  return std::max<double>((std::abs(next_pos - current_pos) * 1.2), 1.0);
//}

void SIGVerseTiagoTeleopKey::operate_hand(bool is_hand_open)
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


void SIGVerseTiagoTeleopKey::show_help()
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

void SIGVerseTiagoTeleopKey::show_help_arm(const std::string &arm_name)
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

  show_help();

  auto logger = node_->get_logger();

  rclcpp::Rate loop_rate(50);

  const float linear_coef  = 0.2f;
  const float angular_coef = 0.5f;

  float move_speed = 1.0f;
  bool is_hand_open = false;

  std::string torso_lift_joint_name = "arm_lift_joint";
  std::string arm_flex_joint_name   = "arm_flex_joint";
  std::string wrist_flex_joint_name = "wrist_flex_joint";

  while (rclcpp::ok())
  {
    if(can_receive(kfd))
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
        case '1':
        {
          send_message(MSG_TELL_ME);
          break;
        }
        case '2':
        {
          send_message(MSG_POINT_IT);
          break;
        }
        case KEYCODE_UP:
        {
          RCLCPP_DEBUG(logger, "Go Forward");
          move_base_twist(+linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_DOWN:
        {
          RCLCPP_DEBUG(logger, "Go Backward");
          move_base_twist(-linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_RIGHT:
        {
          RCLCPP_DEBUG(logger, "Go Right");
          move_base_twist(0.0, 0.0, -angular_coef*move_speed);
          break;
        }
        case KEYCODE_LEFT:
        {
          RCLCPP_DEBUG(logger, "Go Left");
          move_base_twist(0.0, 0.0, +angular_coef*move_speed);
          break;
        }
        case ' ':
        {
          RCLCPP_DEBUG(logger, "Stop");
          move_base_twist(0.0, 0.0, 0.0);
          break;
        }
        case 'r':
        {
          RCLCPP_DEBUG(logger, "Move Speed Up");
          move_speed *= 2;
          if(move_speed > 2  ){ move_speed=2; }
          break;
        }
        case 'f':
        {
          RCLCPP_DEBUG(logger, "Move Speed Down");
          move_speed /= 2;
          if(move_speed < 0.125){ move_speed=0.125; }
          break;
        }
        case 'y':
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Upward");
          operate_arm({ 1.57, -1.4, -3.14, 2.2, 1.57, 0.0, 0.0 }, 3.0);
          break;
        }
        case 'h':
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Horizontal");
          operate_arm({ 1.57, -1.4, -3.14, 2.0, 1.57, 0.6, 0.0 }, 3.0);
          break;
        }
        case 'n':
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Downward");
          operate_arm({ 1.57, -1.3, -3.14, 0.7, 1.57, 0.0, 0.0 }, 3.0);
          break;
        }
        case 'q':
        {
          RCLCPP_DEBUG(logger, "Torso Height - Up");
          operate_torso(0.35, std::max<int>((int)(std::abs(0.35 - torso_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case 'a':
        {
          RCLCPP_DEBUG(logger, "Torso Height - Stop");
          operate_torso(2.0*torso_lift_joint_pos1_-torso_lift_joint_pos2_, 0.5);
          break;
        }
        case 'z':
        {
          RCLCPP_DEBUG(logger, "Torso Height - Down");
          operate_torso(0.0, std::max<int>((int)(std::abs(0.0 - torso_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case 'w':
        {
          RCLCPP_DEBUG(logger, "Turn Head - Left");
          operate_head(+1.24, 0.0, 2.0);
          break;
        }
        case 's':
        {
          RCLCPP_DEBUG(logger, "Turn Head - Front");
          operate_head(0.0, 0.0, 2.0);
          break;
        }
        case 'x':
        {
          RCLCPP_DEBUG(logger, "Turn Head - Right");
          operate_head(-1.24, 0.0, 2.0);
          break;
        }
        case 'e':
        {
          RCLCPP_DEBUG(logger, "Turn Head - Up");
          operate_head(0.0, 0.79, 2.0);
          break;
        }
        case 'd':
        {
          RCLCPP_DEBUG(logger, "Turn Head - Front");
          operate_head(0.0, 0.0, 2.0);
          break;
        }
        case 'c':
        {
          RCLCPP_DEBUG(logger, "Turn Head - Down");
          operate_head(0.0, -0.98, 2.0);
          break;
        }
        case 'u':
        case 'j':
        case 'm':
        case 'i':
        case 'k':
        case 'o':
        case 'l':
        {
          int joint_number;

          switch(c)
          {
            case 'u':{ joint_number = 1; RCLCPP_DEBUG(logger, "Control Arm1"); show_help_arm("Arm1"); break; }
            case 'j':{ joint_number = 2; RCLCPP_DEBUG(logger, "Control Arm2"); show_help_arm("Arm2"); break; }
            case 'm':{ joint_number = 3; RCLCPP_DEBUG(logger, "Control Arm3"); show_help_arm("Arm3"); break; }
            case 'i':{ joint_number = 4; RCLCPP_DEBUG(logger, "Control Arm4"); show_help_arm("Arm4"); break; }
            case 'k':{ joint_number = 5; RCLCPP_DEBUG(logger, "Control Arm5"); show_help_arm("Arm5"); break; }
            case 'o':{ joint_number = 6; RCLCPP_DEBUG(logger, "Control Arm6"); show_help_arm("Arm6"); break; }
            case 'l':{ joint_number = 7; RCLCPP_DEBUG(logger, "Control Arm7"); show_help_arm("Arm7"); break; }
          }

          bool is_in_control = true;

          while(is_in_control)
          {
            rclcpp::spin_some(node_);
            loop_rate.sleep();

            if(!can_receive(kfd)){ continue; }

            if(read(kfd, &c, 1) < 0)
            {
              perror("read():");
              exit(EXIT_FAILURE);
            }

            switch(c)
            {
              case 'u':{ RCLCPP_DEBUG(logger, "Arm  +  "); operate_arm(joint_number, +1.0, 2.0); break; }
              case 'j':{ RCLCPP_DEBUG(logger, "Arm Stop"); operate_arm(joint_number,  0.0, 2.0); break; }
              case 'm':{ RCLCPP_DEBUG(logger, "Arm  -  "); operate_arm(joint_number, -1.0, 2.0); break; }
              case 'q':
              {
                is_in_control = false;
                show_help();
                break;
              }
            }
          }
          break;
        }
        case 'g':
        {
          operate_hand(is_hand_open);

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


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  SIGVerseTiagoTeleopKey tiago_teleop_key;
  return tiago_teleop_key.run();
}