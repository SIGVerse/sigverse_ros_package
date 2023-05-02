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

class SIGVerseAiAgentTeleopKey
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
  static const char KEYCODE_T = 0x74;
  static const char KEYCODE_U = 0x75;
  static const char KEYCODE_W = 0x77;
  static const char KEYCODE_X = 0x78;
  static const char KEYCODE_Y = 0x79;
  static const char KEYCODE_Z = 0x7a;

  static const char KEYCODE_COMMA  = 0x2c;
  static const char KEYCODE_PERIOD = 0x2e;
  static const char KEYCODE_SPACE  = 0x20;

  std::map<std::string, double> joint_state_map_;
  std::map<std::string, double> joint_max_map_;
  std::map<std::string, double> joint_min_map_;

public:
  SIGVerseAiAgentTeleopKey();

  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);

////  void messageCallback(const std_msgs::String::ConstPtr& message);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
////  void sendMessage(const std::string &message);
  void moveBaseTwist(double linear_x, double linear_y, double angular_z);
  void operateRightArm(const double joint1, const double joint2, const double joint3, const double duration_sec);
  void operate(const int joint_number, const double arm_pos, const double duration_sec);
  void operate(const std::vector<double> & positions, const double duration_sec);
  void operateHand(const bool is_right_hand, const double hand_pos, const double duration_sec);

  void showHelp();
  void showHelpHeadController();
  void showHelpArmController(const std::string &arm_name);
  int run();

private:
  ros::NodeHandle node_handle_;

  ros::Subscriber sub_joint_state_;
  ros::Publisher  pub_base_twist_;
  ros::Publisher  pub_trajectory_;
  ros::Publisher  pub_hand_trajectory_;

//  tf::TransformListener listener_;
};


SIGVerseAiAgentTeleopKey::SIGVerseAiAgentTeleopKey()
{
  joint_state_map_["head_controller_joint1"]  = 0.0;
  joint_state_map_["head_pan_joint"]          = 0.0;
  joint_state_map_["head_tilt_joint"]         = 0.0;
  joint_state_map_["right_controller_joint1"] = 0.0;
  joint_state_map_["right_controller_joint2"] = 0.0;
  joint_state_map_["right_controller_joint3"] = 0.0;
  joint_state_map_["right_hand_motor_joint"]  = 0.0;
  joint_state_map_["left_controller_joint1"]  = 0.0;
  joint_state_map_["left_controller_joint2"]  = 0.0;
  joint_state_map_["left_controller_joint3"]  = 0.0;
  joint_state_map_["left_hand_motor_joint"]   = 0.0;

  joint_max_map_["head_controller_joint1"]  = +2.0;
  joint_max_map_["head_pan_joint"]          = +0.785;
  joint_max_map_["head_tilt_joint"]         = +0.785;
  joint_max_map_["right_controller_joint1"] = +1.0;
  joint_max_map_["right_controller_joint2"] = +0.5;
  joint_max_map_["right_controller_joint3"] = +2.0;
  joint_max_map_["right_hand_motor_joint"]  = +1.0;
  joint_max_map_["left_controller_joint1"]  = +1.0;
  joint_max_map_["left_controller_joint2"]  = +1.0;
  joint_max_map_["left_controller_joint3"]  = +2.0;
  joint_max_map_["left_hand_motor_joint"]   = +1.0;

  joint_min_map_["head_controller_joint1"]  = +1.0;
  joint_min_map_["head_pan_joint"]          = -0.785;
  joint_min_map_["head_tilt_joint"]         = -0.785;
  joint_min_map_["right_controller_joint1"] = -0.5;
  joint_min_map_["right_controller_joint2"] = -1.0;
  joint_min_map_["right_controller_joint3"] = +0.0;
  joint_min_map_["right_hand_motor_joint"]  = +0.0;
  joint_min_map_["left_controller_joint1"]  = -0.5;
  joint_min_map_["left_controller_joint2"]  = -0.5;
  joint_min_map_["left_controller_joint3"]  = +0.0;
  joint_min_map_["left_hand_motor_joint"]   = +0.0;
}


void SIGVerseAiAgentTeleopKey::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int SIGVerseAiAgentTeleopKey::canReceive( int fd )
{
  fd_set fdset;
  int ret;
  struct timeval timeout;
  FD_ZERO( &fdset );
  FD_SET( fd , &fdset );

  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  return select( fd+1 , &fdset , NULL , NULL , &timeout );
}

//void SIGVerseAiAgentTeleopKey::messageCallback(const std_msgs::String::ConstPtr& message)
//{
//  ROS_INFO("Subscribe message: %s", message->data.c_str());
//}

void SIGVerseAiAgentTeleopKey::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  for(int i=0; i<joint_state->name.size(); i++)
  {
    if(joint_state->name[i] == "head_controller_joint1") { joint_state_map_["head_controller_joint1"]  = joint_state->position[i]; }
    if(joint_state->name[i] == "head_pan_joint")         { joint_state_map_["head_pan_joint"]          = joint_state->position[i]; }
    if(joint_state->name[i] == "head_tilt_joint")        { joint_state_map_["head_tilt_joint"]         = joint_state->position[i]; }
    if(joint_state->name[i] == "right_controller_joint1"){ joint_state_map_["right_controller_joint1"] = joint_state->position[i]; }
    if(joint_state->name[i] == "right_controller_joint2"){ joint_state_map_["right_controller_joint2"] = joint_state->position[i]; }
    if(joint_state->name[i] == "right_controller_joint3"){ joint_state_map_["right_controller_joint3"] = joint_state->position[i]; }
    if(joint_state->name[i] == "right_hand_motor_joint") { joint_state_map_["right_hand_motor_joint"]  = joint_state->position[i]; }
    if(joint_state->name[i] == "left_controller_joint1") { joint_state_map_["left_controller_joint1"]  = joint_state->position[i]; }
    if(joint_state->name[i] == "left_controller_joint2") { joint_state_map_["left_controller_joint2"]  = joint_state->position[i]; }
    if(joint_state->name[i] == "left_controller_joint3") { joint_state_map_["left_controller_joint3"]  = joint_state->position[i]; }
    if(joint_state->name[i] == "left_hand_motor_joint")  { joint_state_map_["left_hand_motor_joint"]   = joint_state->position[i]; }
  }
}

//void SIGVerseAiAgentTeleopKey::sendMessage(const std::string &message)
//{
//  ROS_INFO("Send message:%s", message.c_str());
//
//  std_msgs::String string_msg;
//  string_msg.data = message;
//  pub_msg_.publish(string_msg);
//}

void SIGVerseAiAgentTeleopKey::moveBaseTwist(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_.publish(twist);
}

void SIGVerseAiAgentTeleopKey::operateRightArm(const double joint1, const double joint2, const double joint3, const double duration_sec)
{
  std::vector<double> positions =
  {
    joint_state_map_["head_controller_joint1"],
    joint_state_map_["head_pan_joint"],
    joint_state_map_["head_tilt_joint"],
    joint_state_map_["right_controller_joint1"],
    joint_state_map_["right_controller_joint2"],
    joint_state_map_["right_controller_joint3"],
    joint_state_map_["left_controller_joint1"],
    joint_state_map_["left_controller_joint2"],
    joint_state_map_["left_controller_joint3"]
  };

  positions[3] = joint1;
  positions[4] = joint2;
  positions[5] = joint3;

  this->operate(positions, duration_sec);
}

void SIGVerseAiAgentTeleopKey::operate(const int joint_number, const double add_pos, const double duration_sec)
{
  std::vector<double> positions =
  {
    joint_state_map_["head_controller_joint1"],
    joint_state_map_["head_pan_joint"],
    joint_state_map_["head_tilt_joint"],
    joint_state_map_["right_controller_joint1"],
    joint_state_map_["right_controller_joint2"],
    joint_state_map_["right_controller_joint3"],
    joint_state_map_["left_controller_joint1"],
    joint_state_map_["left_controller_joint2"],
    joint_state_map_["left_controller_joint3"]
  };

  std::string joint_name = "";

  switch(joint_number)
  {
    case 1:{ joint_name = "head_controller_joint1";  break; }
    case 2:{ joint_name = "head_pan_joint";          break; }
    case 3:{ joint_name = "head_tilt_joint";         break; }
    case 4:{ joint_name = "right_controller_joint1"; break; }
    case 5:{ joint_name = "right_controller_joint2"; break; }
    case 6:{ joint_name = "right_controller_joint3"; break; }
    case 7:{ joint_name = "left_controller_joint1";  break; }
    case 8:{ joint_name = "left_controller_joint2";  break; }
    case 9:{ joint_name = "left_controller_joint3";  break; }
  }
  
  // Clamp
  double clamped_next_pos = std::min(std::max(joint_min_map_[joint_name], joint_state_map_[joint_name]+add_pos), joint_max_map_[joint_name]);

  positions[joint_number-1] = clamped_next_pos;

  this->operate(positions, duration_sec);
}

void SIGVerseAiAgentTeleopKey::operate(const std::vector<double> & positions, const double duration_sec)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("head_controller_joint1");
  joint_trajectory.joint_names.push_back("head_pan_joint");
  joint_trajectory.joint_names.push_back("head_tilt_joint");
  joint_trajectory.joint_names.push_back("right_controller_joint1");
  joint_trajectory.joint_names.push_back("right_controller_joint2");
  joint_trajectory.joint_names.push_back("right_controller_joint3");
  joint_trajectory.joint_names.push_back("left_controller_joint1");
  joint_trajectory.joint_names.push_back("left_controller_joint2");
  joint_trajectory.joint_names.push_back("left_controller_joint3");
  
  trajectory_msgs::JointTrajectoryPoint joint_point;

  joint_point.positions = positions;

  joint_point.time_from_start = ros::Duration(duration_sec);
  joint_trajectory.points.push_back(joint_point);
  pub_trajectory_.publish(joint_trajectory);
}


void SIGVerseAiAgentTeleopKey::operateHand(const bool is_right_hand, const double hand_pos, const double duration_sec)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("right_hand_motor_joint");
  joint_trajectory.joint_names.push_back("left_hand_motor_joint");

  trajectory_msgs::JointTrajectoryPoint hand_joint_point;

  hand_joint_point.positions = { joint_state_map_["right_hand_motor_joint"], joint_state_map_["left_hand_motor_joint"] };

  if(is_right_hand)
  {
	hand_joint_point.positions[0] = hand_pos;
  }
  else
  {
	hand_joint_point.positions[1] = hand_pos;
  }
  
  hand_joint_point.time_from_start = ros::Duration(duration_sec);
  joint_trajectory.points.push_back(hand_joint_point);
  pub_hand_trajectory_.publish(joint_trajectory);
}


void SIGVerseAiAgentTeleopKey::showHelp()
{
  puts("Operate by Keyboard");
  puts("---------------------------");
  puts("Arrow Keys/WASD : Move AI Agent");
  puts("SPACE           : Stop AI Agent");
  puts("I/K : Increase/Decrease Moving Speed");
  puts("---------------------------");
  puts("U/J : Up/Down Right Arm");
  puts("G/O : Grasp/Open Right Hand");
  puts("---------------------------");
  puts("H : Control Head Controller");
  puts("R : Control Right Controller ");
  puts("L : Control Left Controller ");
  puts("---------------------------");
}

void SIGVerseAiAgentTeleopKey::showHelpHeadController()
{
  puts("");
  puts("---------------------------");
  puts("Control Head");
  puts("W/X    : Move Up/Down ");
  puts("D/A    : Pan  Right/Left ");
  puts("U/M    : Tilt Up/Down ");
  puts("S/SPACE: Stop");
  puts("---------------------------");
  puts("Q : Quit");
  puts("");
}

void SIGVerseAiAgentTeleopKey::showHelpArmController(const std::string &controller_name)
{
  puts("");
  puts("---------------------------");
  puts(("Control "+controller_name).c_str());
  puts("W/X    : Move Up/Down ");
  puts("D/A    : Move Right/Left ");
  puts("U/M    : Move Forward/Back ");
  puts("G/O    : Grasp/Open");
  puts("S/SPACE: Stop");
  puts("---------------------------");
  puts("Q : Quit");
  puts("");
}


int SIGVerseAiAgentTeleopKey::run()
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
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(40);

  std::string sub_joint_state_topic_name;
  std::string pub_base_twist_topic_name;
  std::string pub_trajectory_topic_name;
  std::string pub_hand_trajectory_topic_name;
  
  node_handle_.param<std::string>("ai_agent_teleop_key/sub_joint_state_topic_name",     sub_joint_state_topic_name,     "/joint_states");
  node_handle_.param<std::string>("ai_agent_teleop_key/pub_base_twist_topic_name",      pub_base_twist_topic_name,      "/command_velocity");
  node_handle_.param<std::string>("ai_agent_teleop_key/pub_trajectory_topic_name",      pub_trajectory_topic_name,      "/controller/command");
  node_handle_.param<std::string>("ai_agent_teleop_key/pub_hand_trajectory_topic_name", pub_hand_trajectory_topic_name, "/hand_controller/command");

  sub_joint_state_     = node_handle_.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &SIGVerseAiAgentTeleopKey::jointStateCallback, this);
  pub_base_twist_      = node_handle_.advertise<geometry_msgs::Twist>(pub_base_twist_topic_name, 10);
  pub_trajectory_      = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_trajectory_topic_name, 10);
  pub_hand_trajectory_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_hand_trajectory_topic_name, 10);

  const float linear_coef  = 1.0f;
  const float angular_coef = 1.0f;

  float move_speed = 0.5f;
  bool is_hand_open = false;

  while (ros::ok())
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
        case KEYCODE_W:
        case KEYCODE_UP:
        {
          ROS_DEBUG("Go Forward");
          moveBaseTwist(+linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_S:
        case KEYCODE_DOWN:
        {
          ROS_DEBUG("Go Backward");
          moveBaseTwist(-linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_D:
        case KEYCODE_RIGHT:
        {
          ROS_DEBUG("Go Right");
          moveBaseTwist(0.0, 0.0, -angular_coef*move_speed);
          break;
        }
        case KEYCODE_A:
        case KEYCODE_LEFT:
        {
          ROS_DEBUG("Go Left");
          moveBaseTwist(0.0, 0.0, +angular_coef*move_speed);
          break;
        }
        case KEYCODE_SPACE:
        {
          ROS_DEBUG("Stop");
          moveBaseTwist(0.0, 0.0, 0.0);
          break;
        }
        case KEYCODE_I:
        {
          ROS_DEBUG("Move Speed Up");
          move_speed *= 2;
          if(move_speed > 2  ){ move_speed=2; }
          break;
        }
        case KEYCODE_K:
        {
          ROS_DEBUG("Move Speed Down");
          move_speed /= 2;
          if(move_speed < 0.125){ move_speed=0.125; }
          break;
        }
        case KEYCODE_U:
        {
          ROS_DEBUG("Up Right Arm");
          // The height of the arm is changed according to the height of the head.
          operateRightArm(0.55, 0.0, 1.17, 1.0);
          break;
        }
        case KEYCODE_J:
        {
          ROS_DEBUG("Down Right Arm");
          // The height of the arm is changed according to the height of the head.
          operateRightArm(0.2, -0.2, joint_state_map_["head_controller_joint1"]/2, 1.0);
          break;
        }
        case KEYCODE_G:
        {
          ROS_DEBUG("Grasp (Right Hand)");
          operateHand(true, 1.0, 1.0);
          break;
        }
        case KEYCODE_O:
        {
          ROS_DEBUG("Open (Right Hand)");
          operateHand(true, 0.0, 1.0);
          break;
        }
        case KEYCODE_H:
        {
          ROS_DEBUG("Control Head Controller");
          showHelpHeadController();

          bool is_in_control = true;

          int joint_number_base = 0;
          
          while(is_in_control)
          {
            ros::spinOnce();
            loop_rate.sleep();

            if(!canReceive(kfd)){ continue; }

            if(read(kfd, &c, 1) < 0)
            {
              perror("read():");
              exit(EXIT_FAILURE);
            }

            switch(c)
            {
              case KEYCODE_W:{ ROS_DEBUG("Move Up");   operate(joint_number_base+1, +0.5, 2.0); break; }
              case KEYCODE_X:{ ROS_DEBUG("Move Down"); operate(joint_number_base+1, -0.5, 2.0); break; }
              case KEYCODE_D:{ ROS_DEBUG("Pan Right"); operate(joint_number_base+2, -0.7, 1.0); break; }
              case KEYCODE_A:{ ROS_DEBUG("Pan Left");  operate(joint_number_base+2, +0.7, 1.0); break; }
              case KEYCODE_U:{ ROS_DEBUG("Tilt Up");   operate(joint_number_base+3, -0.7, 1.0); break; }
              case KEYCODE_M:{ ROS_DEBUG("Tilt Down"); operate(joint_number_base+3, +0.7, 1.0); break; }
              case KEYCODE_SPACE:
              case KEYCODE_S:{ ROS_DEBUG("Stop");      operate(1,                   +0.0, 1.0); break; }
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
        case KEYCODE_R:
        case KEYCODE_L:
        {
          int joint_number_base;
          bool is_right_hand;
          
          switch(c)
          {
            case KEYCODE_R:{ joint_number_base = 3; is_right_hand = true;  ROS_DEBUG("Control Right Arm"); showHelpArmController("Right Arm");  break; }
            case KEYCODE_L:{ joint_number_base = 6; is_right_hand = false; ROS_DEBUG("Control Left Arm");  showHelpArmController("Left Arm");   break; }
          }

          bool is_in_control = true;

          while(is_in_control)
          {
            ros::spinOnce();
            loop_rate.sleep();

            if(!canReceive(kfd)){ continue; }

            if(read(kfd, &c, 1) < 0)
            {
              perror("read():");
              exit(EXIT_FAILURE);
            }

            switch(c)
            {
              case KEYCODE_W:{ ROS_DEBUG("Move Up");      operate(joint_number_base+3, +0.5, 2.0); break; }
              case KEYCODE_X:{ ROS_DEBUG("Move Down");    operate(joint_number_base+3, -0.5, 2.0); break; }
              case KEYCODE_D:{ ROS_DEBUG("Move Right");   operate(joint_number_base+2, -0.5, 2.0); break; }
              case KEYCODE_A:{ ROS_DEBUG("Move Left");    operate(joint_number_base+2, +0.5, 2.0); break; }
              case KEYCODE_U:{ ROS_DEBUG("Move Forward"); operate(joint_number_base+1, +0.5, 2.0); break; }
              case KEYCODE_M:{ ROS_DEBUG("Move Back");    operate(joint_number_base+1, -0.5, 2.0); break; }
              case KEYCODE_G:{ ROS_DEBUG("Grasp");        operateHand(is_right_hand, 1.0, 1.0);    break; }
              case KEYCODE_O:{ ROS_DEBUG("Open Hand");    operateHand(is_right_hand, 0.0, 1.0);    break; }
              case KEYCODE_SPACE:
              case KEYCODE_S:{ ROS_DEBUG("Stop");         operate(1,                   +0.0, 1.0); break; }
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
      }
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  /////////////////////////////////////////////
  // cooked mode
  tcsetattr(kfd, TCSANOW, &cooked);
  /////////////////////////////////////////////

  return EXIT_SUCCESS;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ai_agent_teleop_key");
  SIGVerseAiAgentTeleopKey ai_agent_teleop_key;
  return ai_agent_teleop_key.run();
}
