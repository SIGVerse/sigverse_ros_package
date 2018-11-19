#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class SIGVerseHsrTeleopKey
{
private:
  static const char KEYCODE_1 = 0x31;
  static const char KEYCODE_2 = 0x32;

  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;

  static const char KEYCODE_A = 0x61;
  static const char KEYCODE_B = 0x62;
  static const char KEYCODE_C = 0x63;
  static const char KEYCODE_D = 0x64;
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
  static const char KEYCODE_S = 0x73;
  static const char KEYCODE_U = 0x75;
  static const char KEYCODE_Y = 0x79;
  static const char KEYCODE_Z = 0x7a;

  static const char KEYCODE_COMMA  = 0x2c;
  static const char KEYCODE_PERIOD = 0x2e;

  const std::string ARM_LIFT_JOINT_NAME = "arm_lift_joint";

  const std::string MSG_TELL_ME  = "Please tell me";
  const std::string MSG_POINT_IT = "Please point it";

public:
  SIGVerseHsrTeleopKey();

  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);

  void messageCallback(const std_msgs::String::ConstPtr& message);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  void sendMessage(ros::Publisher &publisher, const std::string &message);
  void moveBase(ros::Publisher &publisher, double linear_x, double linear_y, double angular_z);
  void moveArm(ros::Publisher &publisher, const std::string &name, const double position, const int duration_sec);
  void moveHand(ros::Publisher &publisher, bool grasp);

  void showHelp();
  int run(int argc, char **argv);

private:
  // Last position and previous position of arm_lift_joint
  double arm_lift_joint_pos1_;
  double arm_lift_joint_pos2_;
};


SIGVerseHsrTeleopKey::SIGVerseHsrTeleopKey()
{
  arm_lift_joint_pos1_ = 0.0;
  arm_lift_joint_pos2_ = 0.0;
}


void SIGVerseHsrTeleopKey::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int SIGVerseHsrTeleopKey::canReceive( int fd )
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

void SIGVerseHsrTeleopKey::messageCallback(const std_msgs::String::ConstPtr& message)
{
  ROS_INFO("Subscribe message: %s", message->data.c_str());
}

void SIGVerseHsrTeleopKey::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  for(int i=0; i<joint_state->name.size(); i++)
  {
    if(joint_state->name[i]==ARM_LIFT_JOINT_NAME)
    {
      arm_lift_joint_pos2_ = arm_lift_joint_pos1_;
      arm_lift_joint_pos1_ = joint_state->position[i];
//    ROS_INFO("lift:%lf",armLiftJointPos_);
      return;
    }
  }
}

void SIGVerseHsrTeleopKey::sendMessage(ros::Publisher &publisher, const std::string &message)
{
  ROS_INFO("Send message:%s", message.c_str());

  std_msgs::String string_msg;
  string_msg.data = message;
  publisher.publish(string_msg);
}

void SIGVerseHsrTeleopKey::moveBase(ros::Publisher &publisher, double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.linear.z  = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_z;

  publisher.publish(twist);
}

void SIGVerseHsrTeleopKey::moveArm(ros::Publisher &publisher, const std::string &name, const double position, const int duration_sec)
{
  std::vector<double> positions;
  positions.push_back(position);

  ros::Duration duration;
  duration.sec = duration_sec;

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("arm_lift_joint");
  joint_trajectory.joint_names.push_back("arm_flex_joint");
  joint_trajectory.joint_names.push_back("arm_roll_joint");
  joint_trajectory.joint_names.push_back("wrist_flex_joint");
  joint_trajectory.joint_names.push_back("wrist_roll_joint");

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;
  if(name == "arm_lift_joint"){        arm_joint_point.positions = {position, 0.0f, 0.0f, 0.0f, 0.0f}; }
  else if(name == "arm_flex_joint"){   arm_joint_point.positions = {0.0f, position, 0.0f, 0.0f, 0.0f}; }
  else if(name == "arm_roll_joint"){   arm_joint_point.positions = {0.0f, 0.0f, position, 0.0f, 0.0f}; }
  else if(name == "wrist_flex_joint"){ arm_joint_point.positions = {0.0f, 0.0f, 0.0f, position, 0.0f}; }
  else if(name == "wrist_roll_joint"){ arm_joint_point.positions = {0.0f, 0.0f, 0.0f, 0.0f, position}; }
  arm_joint_point.time_from_start = duration;
  joint_trajectory.points.push_back(arm_joint_point);

  publisher.publish(joint_trajectory);
}

void SIGVerseHsrTeleopKey::moveHand(ros::Publisher &publisher, bool is_hand_open)
{
  std::vector<std::string> joint_names {"hand_motor_joint"};

  std::vector<double> positions;

  if(is_hand_open)
  {
    ROS_DEBUG("Grasp");
    positions.push_back(-0.105);
  }
  else
  {
    ROS_DEBUG("Open hand");
    positions.push_back(+1.239);
  }

  ros::Duration duration;
  duration.sec = 2;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = duration;

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);

  publisher.publish(joint_trajectory);
}


void SIGVerseHsrTeleopKey::showHelp()
{
  puts("Operate by Keyboard");
  puts("---------------------------");
  puts("arrow keys : Move HSR");
  puts("s          : Stop HSR");
  puts("---------------------------");
  puts("Move HSR Linearly:");
  puts("  u   i   o  ");
  puts("  j   k   l  ");
  puts("  m   ,   .  ");
  puts("---------------------------");
  puts("q/z : Increase/Decrease Moving Speed");
  puts("---------------------------");
  puts("y : Move Arm Up");
  puts("h : Stop Arm");
  puts("n : Move Arm Down");
  puts("---------------------------");
  puts("a : Rotate Arm - Vertical");
  puts("b : Rotate Arm - Upward");
  puts("c : Rotate Arm - Horizontal");
  puts("d : Rotate Arm - Downward");
  puts("---------------------------");
  puts("g : Grasp/Open Hand");
  puts("---------------------------");
  puts(("1 : Send Message: "+MSG_TELL_ME).c_str());
  puts(("2 : Send Message: "+MSG_POINT_IT).c_str());
}


int SIGVerseHsrTeleopKey::run(int argc, char **argv)
{
  char c;

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

  ros::init(argc, argv, "hsr_teleop_key");

  ros::NodeHandle node_handle;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(40);

  std::string sub_msg_to_robot_topic_name;
  std::string pub_msg_to_human_topic_name;
  std::string sub_joint_state_topic_name;
  std::string pub_base_twist_topic_name;
  std::string pub_arm_trajectory_topic_name;
  std::string pub_gripper_trajectory_topic_name;

  node_handle.param<std::string>("sub_msg_to_robot_topic_name",   sub_msg_to_robot_topic_name,   "/hsrb/message/to_robot");
  node_handle.param<std::string>("pub_msg_to_human_topic_name",   pub_msg_to_human_topic_name,   "/hsrb/message/to_human");

  node_handle.param<std::string>("sub_joint_state_topic_name",        sub_joint_state_topic_name,        "/hsrb/joint_states");
  node_handle.param<std::string>("pub_base_twist_topic_name",         pub_base_twist_topic_name,         "/hsrb/command_velocity");
  node_handle.param<std::string>("pub_arm_trajectory_topic_name",     pub_arm_trajectory_topic_name,     "/hsrb/arm_trajectory_controller/command");
  node_handle.param<std::string>("pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name, "/hsrb/gripper_controller/command");

  ros::Subscriber sub_msg                = node_handle.subscribe<std_msgs::String>(sub_msg_to_robot_topic_name, 100, &SIGVerseHsrTeleopKey::messageCallback, this);
  ros::Publisher  pub_msg                = node_handle.advertise<std_msgs::String>(pub_msg_to_human_topic_name, 10);
  ros::Subscriber sub_joint_state        = node_handle.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &SIGVerseHsrTeleopKey::jointStateCallback, this);
  ros::Publisher  pub_base_twist         = node_handle.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
  ros::Publisher  pub_arm_trajectory     = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
  ros::Publisher  pub_gripper_trajectory = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);


  const float linear_coef         = 0.2f;
  const float linear_oblique_coef = 0.141f;
  const float angular_coef        = 0.5f;

  float move_speed = 1.0f;
  bool is_hand_open = false;

  std::string arm_lift_joint_name   = "arm_lift_joint";
  std::string arm_flex_joint_name   = "arm_flex_joint";
  std::string wrist_flex_joint_name = "wrist_flex_joint";

  while (ros::ok())
  {
    if(canReceive(kfd))
    {
      // get the next event from the keyboard
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(EXIT_FAILURE);
      }

      switch(c)
      {
        case KEYCODE_1:
        {
          sendMessage(pub_msg, MSG_TELL_ME);
          break;
        }
        case KEYCODE_2:
        {
          sendMessage(pub_msg, MSG_POINT_IT);
          break;
        }
        case KEYCODE_UP:
        {
          ROS_DEBUG("Go Forward");
          moveBase(pub_base_twist, +linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_DOWN:
        {
          ROS_DEBUG("Go Backward");
          moveBase(pub_base_twist, -linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_RIGHT:
        {
          ROS_DEBUG("Go Right");
          moveBase(pub_base_twist, 0.0, 0.0, -angular_coef*move_speed);
          break;
        }
        case KEYCODE_LEFT:
        {
          ROS_DEBUG("Go Left");
          moveBase(pub_base_twist, 0.0, 0.0, +angular_coef*move_speed);
          break;
        }
        case KEYCODE_S:
        {
          ROS_DEBUG("Stop");
          moveBase(pub_base_twist, 0.0, 0.0, 0.0);
          break;
        }
        case KEYCODE_U:
        {
          ROS_DEBUG("Move Left Forward");
          moveBase(pub_base_twist, +linear_oblique_coef*move_speed, +linear_oblique_coef*move_speed, 0.0);
          break;
        }
        case KEYCODE_I:
        {
          ROS_DEBUG("Move Forward");
          moveBase(pub_base_twist, +linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_O:
        {
          ROS_DEBUG("Move Right Forward");
          moveBase(pub_base_twist, +linear_oblique_coef*move_speed, -linear_oblique_coef*move_speed, 0.0);
          break;
        }
        case KEYCODE_J:
        {
          ROS_DEBUG("Move Left");
          moveBase(pub_base_twist, 0.0, +linear_coef*move_speed, 0.0);
          break;
        }
        case KEYCODE_K:
        {
          ROS_DEBUG("Stop");
          moveBase(pub_base_twist, 0.0, 0.0, 0.0);
          break;
        }
        case KEYCODE_L:
        {
          ROS_DEBUG("Move Right");
          moveBase(pub_base_twist, 0.0, -linear_coef*move_speed, 0.0);
          break;
        }
        case KEYCODE_M:
        {
          ROS_DEBUG("Move Left Backward");
          moveBase(pub_base_twist, -linear_oblique_coef*move_speed, +linear_oblique_coef*move_speed, 0.0);
          break;
        }
        case KEYCODE_COMMA:
        {
          ROS_DEBUG("Move Backward");
          moveBase(pub_base_twist, -linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_PERIOD:
        {
          ROS_DEBUG("Move Right Backward");
          moveBase(pub_base_twist, -linear_oblique_coef*move_speed, -linear_oblique_coef*move_speed, 0.0);
          break;
        }
        case KEYCODE_Q:
        {
          ROS_DEBUG("Move Speed Up");
          move_speed *= 2;
          if(move_speed > 2  ){ move_speed=2; }
          break;
        }
        case KEYCODE_Z:
        {
          ROS_DEBUG("Move Speed Down");
          move_speed /= 2;
          if(move_speed < 0.125){ move_speed=0.125; }
          break;
        }
        case KEYCODE_Y:
        {
          ROS_DEBUG("Move Arm Up");
          moveArm(pub_arm_trajectory, arm_lift_joint_name, 0.69, std::max<int>((int)(std::abs(0.69 - arm_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case KEYCODE_H:
        {
          ROS_DEBUG("Stop Arm");
          moveArm(pub_arm_trajectory, arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
          break;
        }
        case KEYCODE_N:
        {
          ROS_DEBUG("Move Arm Down");
          moveArm(pub_arm_trajectory, arm_lift_joint_name, 0.0, std::max<int>((int)(std::abs(0.0 - arm_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case KEYCODE_A:
        {
          ROS_DEBUG("Rotate Arm - Vertical");
          moveArm(pub_arm_trajectory, arm_flex_joint_name, 0.0, 1);
          moveArm(pub_arm_trajectory, wrist_flex_joint_name, -1.57, 1);
          break;
        }
        case KEYCODE_B:
        {
          ROS_DEBUG("Rotate Arm - Upward");
          moveArm(pub_arm_trajectory, arm_flex_joint_name, -0.785, 1);
          moveArm(pub_arm_trajectory, wrist_flex_joint_name, -0.785, 1);
          break;
        }
        case KEYCODE_C:
        {
          ROS_DEBUG("Rotate Arm - Horizontal");
          moveArm(pub_arm_trajectory, arm_flex_joint_name, -1.57, 1);
          moveArm(pub_arm_trajectory, wrist_flex_joint_name, 0.0, 1);
          break;
        }
        case KEYCODE_D:
        {
          ROS_DEBUG("Rotate Arm - Downward");
          moveArm(pub_arm_trajectory, arm_flex_joint_name, -2.2, 1);
          moveArm(pub_arm_trajectory, wrist_flex_joint_name, 0.35, 1);
          break;
        }
        case KEYCODE_G:
        {
          moveHand(pub_gripper_trajectory, is_hand_open);

          is_hand_open = !is_hand_open;
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
  SIGVerseHsrTeleopKey hsr_teleop_key;
  return hsr_teleop_key.run(argc, argv);
}
