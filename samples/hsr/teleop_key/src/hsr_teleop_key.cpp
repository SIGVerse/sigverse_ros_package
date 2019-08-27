#include <stdio.h>
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

public:
  SIGVerseHsrTeleopKey();

  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);

  void messageCallback(const std_msgs::String::ConstPtr& message);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  void sendMessage(const std::string &message);
  void moveBaseTwist(double linear_x, double linear_y, double angular_z);
  void moveBaseJointTrajectory(double linear_x, double linear_y, double theta, double duration_sec);
  void operateArm(const double arm_lift_pos, const double arm_flex_pos, const double wrist_flex_pos, const double duration_sec);
  void operateArm(const std::string &name, const double position, const double duration_sec);
  void operateArmFlex(const double arm_flex_pos, const double wrist_flex_pos);
  double getDurationRot(const double next_pos, const double current_pos);
  void moveHand(bool grasp);

  void showHelp();
  int run();

private:
  bool  use_simple_command_;
  
  // Last position and previous position of arm_lift_joint
  double arm_lift_joint_pos1_;
  double arm_lift_joint_pos2_;
  double arm_flex_joint_pos_;
  double wrist_flex_joint_pos_;

  ros::NodeHandle node_handle_;

  ros::Subscriber sub_msg_;
  ros::Publisher  pub_msg_;
  ros::Subscriber sub_joint_state_;
  ros::Publisher  pub_base_twist_;
  ros::Publisher  pub_base_trajectory_;
  ros::Publisher  pub_arm_trajectory_;
  ros::Publisher  pub_gripper_trajectory_;

  tf::TransformListener listener_;
};


SIGVerseHsrTeleopKey::SIGVerseHsrTeleopKey()
{
  use_simple_command_ = true;
  arm_lift_joint_pos1_   = 0.0;
  arm_lift_joint_pos2_   = 0.0;
  arm_flex_joint_pos_    = 0.0;
  wrist_flex_joint_pos_  = 0.0;
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
    if(joint_state->name[i] == "arm_lift_joint")
    {
      arm_lift_joint_pos2_ = arm_lift_joint_pos1_;
      arm_lift_joint_pos1_ = joint_state->position[i];
    }
    if(joint_state->name[i] == "arm_flex_joint")
    {
      arm_flex_joint_pos_ = joint_state->position[i];
    }
    if(joint_state->name[i] == "wrist_flex_joint")
    {
      wrist_flex_joint_pos_ = joint_state->position[i];
    }
  }
}

void SIGVerseHsrTeleopKey::sendMessage(const std::string &message)
{
  ROS_INFO("Send message:%s", message.c_str());

  std_msgs::String string_msg;
  string_msg.data = message;
  pub_msg_.publish(string_msg);
}

void SIGVerseHsrTeleopKey::moveBaseTwist(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_.publish(twist);
}

void SIGVerseHsrTeleopKey::moveBaseJointTrajectory(double linear_x, double linear_y, double theta, double duration_sec)
{
  if(listener_.canTransform("/odom", "/base_footprint", ros::Time(0)) == false)
  {
    return;
  }

  geometry_msgs::PointStamped basefootprint_2_target;
  geometry_msgs::PointStamped odom_2_target;
  basefootprint_2_target.header.frame_id = "/base_footprint";
  basefootprint_2_target.header.stamp = ros::Time(0);
  basefootprint_2_target.point.x = linear_x;
  basefootprint_2_target.point.y = linear_y;
  listener_.transformPoint("/odom", basefootprint_2_target, odom_2_target);

  tf::StampedTransform transform;
  listener_.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
  tf::Quaternion currentRotation = transform.getRotation();
  tf::Matrix3x3 mat(currentRotation);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("odom_x");
  joint_trajectory.joint_names.push_back("odom_y");
  joint_trajectory.joint_names.push_back("odom_t");

  trajectory_msgs::JointTrajectoryPoint omni_joint_point;
  omni_joint_point.positions = {odom_2_target.point.x, odom_2_target.point.y, yaw + theta};
  omni_joint_point.time_from_start = ros::Duration(duration_sec);

  joint_trajectory.points.push_back(omni_joint_point);
  pub_base_trajectory_.publish(joint_trajectory);
}

void SIGVerseHsrTeleopKey::operateArm(const double arm_lift_pos, const double arm_flex_pos, const double wrist_flex_pos, const double duration_sec)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("arm_lift_joint");
  joint_trajectory.joint_names.push_back("arm_flex_joint");
  joint_trajectory.joint_names.push_back("arm_roll_joint");
  joint_trajectory.joint_names.push_back("wrist_flex_joint");
  joint_trajectory.joint_names.push_back("wrist_roll_joint");

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;

  arm_joint_point.positions = {arm_lift_pos, arm_flex_pos, 0.0f, wrist_flex_pos, 0.0f};

  arm_joint_point.time_from_start = ros::Duration(duration_sec);
  joint_trajectory.points.push_back(arm_joint_point);
  pub_arm_trajectory_.publish(joint_trajectory);
}

void SIGVerseHsrTeleopKey::operateArm(const std::string &name, const double position, const double duration_sec)
{
  if(name == "arm_lift_joint")
  {
    this->operateArm(position, arm_flex_joint_pos_, wrist_flex_joint_pos_, duration_sec);
  }
  else if(name == "arm_flex_joint")
  {
    this->operateArm(2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, position, wrist_flex_joint_pos_, duration_sec);
  }
  else if(name == "wrist_flex_joint")
  {
    this->operateArm(2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, arm_flex_joint_pos_, position, duration_sec);
  }
}

void SIGVerseHsrTeleopKey::operateArmFlex(const double arm_flex_pos, const double wrist_flex_pos)
{
  double duration = std::max(this->getDurationRot(arm_flex_pos, arm_flex_joint_pos_), this->getDurationRot(wrist_flex_pos, wrist_flex_joint_pos_));

  this->operateArm(2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, arm_flex_pos, wrist_flex_pos, duration);
}

double SIGVerseHsrTeleopKey::getDurationRot(const double next_pos, const double current_pos)
{
  return std::max<double>((std::abs(next_pos - current_pos) * 1.2), 1.0);
}

void SIGVerseHsrTeleopKey::moveHand(bool is_hand_open)
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

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = ros::Duration(2);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);
  pub_gripper_trajectory_.publish(joint_trajectory);
}


void SIGVerseHsrTeleopKey::showHelp()
{
  puts("Operate by Keyboard");
  puts("---------------------------");
  puts("arrow keys : Move HSR");
  puts("space      : Stop HSR");
  puts("r/f : Increase/Decrease Moving Speed");

  if(!use_simple_command_)
  {
    puts("---------------------------");
    puts("Move HSR Linearly (1m):");
    puts("  u   i   o  ");
    puts("  j   k   l  ");
    puts("  m   ,   .  ");
  }
  
  puts("---------------------------");
  puts("y : Rotate Arm - Vertical");
  puts("h : Rotate Arm - Upward");
  puts("n : Rotate Arm - Downward");
  puts("---------------------------");
  puts("q/a/z : Up/Stop/Down Torso");
  puts("w/s/x : Up/Stop/Down Arm Flex Angle");
  puts("e/d/c : Up/Stop/Down Arm Wrist Angle");
  puts("---------------------------");
  puts("g : Grasp/Open Hand");
  puts("---------------------------");
  puts(("1 : Send Message: "+MSG_TELL_ME).c_str());
  puts(("2 : Send Message: "+MSG_POINT_IT).c_str());
}


int SIGVerseHsrTeleopKey::run()
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

  node_handle_.param<bool>("hsr_teleop_key/use_simple_command", use_simple_command_, true);
  
  showHelp();

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(40);

  std::string sub_msg_to_robot_topic_name;
  std::string pub_msg_to_human_topic_name;
  std::string sub_joint_state_topic_name;
  std::string pub_base_twist_topic_name;
  std::string pub_base_trajectory_topic_name;
  std::string pub_arm_trajectory_topic_name;
  std::string pub_gripper_trajectory_topic_name;

  node_handle_.param<std::string>("hsr_teleop_key/sub_msg_to_robot_topic_name",       sub_msg_to_robot_topic_name,       "/hsrb/message/to_robot");
  node_handle_.param<std::string>("hsr_teleop_key/pub_msg_to_human_topic_name",       pub_msg_to_human_topic_name,       "/hsrb/message/to_human");
  node_handle_.param<std::string>("hsr_teleop_key/sub_joint_state_topic_name",        sub_joint_state_topic_name,        "/hsrb/joint_states");
  node_handle_.param<std::string>("hsr_teleop_key/pub_base_twist_topic_name",         pub_base_twist_topic_name,         "/hsrb/command_velocity");
  node_handle_.param<std::string>("hsr_teleop_key/pub_base_trajectory_topic_name",    pub_base_trajectory_topic_name,    "/hsrb/omni_base_controller/command");
  node_handle_.param<std::string>("hsr_teleop_key/pub_arm_trajectory_topic_name",     pub_arm_trajectory_topic_name,     "/hsrb/arm_trajectory_controller/command");
  node_handle_.param<std::string>("hsr_teleop_key/pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name, "/hsrb/gripper_controller/command");

  sub_msg_                = node_handle_.subscribe<std_msgs::String>(sub_msg_to_robot_topic_name, 100, &SIGVerseHsrTeleopKey::messageCallback, this);
  pub_msg_                = node_handle_.advertise<std_msgs::String>(pub_msg_to_human_topic_name, 10);
  sub_joint_state_        = node_handle_.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &SIGVerseHsrTeleopKey::jointStateCallback, this);
  pub_base_twist_         = node_handle_.advertise<geometry_msgs::Twist>(pub_base_twist_topic_name, 10);
  pub_base_trajectory_    = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_base_trajectory_topic_name, 10);
  pub_arm_trajectory_     = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
  pub_gripper_trajectory_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);

  const float linear_coef  = 0.2f;
  const float angular_coef = 0.5f;

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
          ROS_DEBUG("Go Forward");
          moveBaseTwist(+linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_DOWN:
        {
          ROS_DEBUG("Go Backward");
          moveBaseTwist(-linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_RIGHT:
        {
          ROS_DEBUG("Go Right");
          moveBaseTwist(0.0, 0.0, -angular_coef*move_speed);
          break;
        }
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
        case KEYCODE_U:
        {
          if(use_simple_command_){ break; }

          ROS_DEBUG("Move Left Forward");
          moveBaseJointTrajectory(+1.0, +1.0, +M_PI_4, 10);
          break;
        }
        case KEYCODE_I:
        {
          if(use_simple_command_){ break; }

          ROS_DEBUG("Move Forward");
          moveBaseJointTrajectory(+1.0, 0.0, 0.0, 10);
          break;
        }
        case KEYCODE_O:
        {
          if(use_simple_command_){ break; }

          ROS_DEBUG("Move Right Forward");
          moveBaseJointTrajectory(+1.0, -1.0, -M_PI_4, 10);
          break;
        }
        case KEYCODE_J:
        {
          if(use_simple_command_){ break; }

          ROS_DEBUG("Move Left");
          moveBaseJointTrajectory(0.0, +1.0, +M_PI_2, 10);
          break;
        }
        case KEYCODE_K:
        {
          if(use_simple_command_){ break; }

          ROS_DEBUG("Stop");
          moveBaseJointTrajectory(0.0, 0.0, 0.0, 0.5);
          break;
        }
        case KEYCODE_L:
        {
          if(use_simple_command_){ break; }

          ROS_DEBUG("Move Right");
          moveBaseJointTrajectory(0.0, -1.0, -M_PI_2, 10);
          break;
        }
        case KEYCODE_M:
        {
          if(use_simple_command_){ break; }

          ROS_DEBUG("Move Left Backward");
          moveBaseJointTrajectory(-1.0, +1.0, +M_PI_2+M_PI_4, 10);
          break;
        }
        case KEYCODE_COMMA:
        {
          if(use_simple_command_){ break; }

          ROS_DEBUG("Move Backward");
          moveBaseJointTrajectory(-1.0, 0.0, +M_PI, 10);
          break;
        }
        case KEYCODE_PERIOD:
        {
          if(use_simple_command_){ break; }

          ROS_DEBUG("Move Right Backward");
          moveBaseJointTrajectory(-1.0, -1.0, -M_PI_2-M_PI_4, 10);
          break;
        }
        case KEYCODE_R:
        {
          ROS_DEBUG("Move Speed Up");
          move_speed *= 2;
          if(move_speed > 2  ){ move_speed=2; }
          break;
        }
        case KEYCODE_F:
        {
          ROS_DEBUG("Move Speed Down");
          move_speed /= 2;
          if(move_speed < 0.125){ move_speed=0.125; }
          break;
        }
        case KEYCODE_Y:
        {
          ROS_DEBUG("Rotate Arm - Vertical");
          operateArmFlex(0.0, -1.57);
          break;
        }
        case KEYCODE_H:
        {
          ROS_DEBUG("Rotate Arm - Upward");
          operateArmFlex(-0.8, -0.77);
          break;
        }
        case KEYCODE_N:
        {
          ROS_DEBUG("Rotate Arm - Downward");
          operateArmFlex(-2.2, 0.35);
          break;
        }
        case KEYCODE_Q:
        {
          ROS_DEBUG("Torso Height - Up");
          operateArm(arm_lift_joint_name, 0.69, std::max<int>((int)(std::abs(0.69 - arm_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case KEYCODE_A:
        {
          ROS_DEBUG("Torso Height - Stop");
          operateArm(arm_lift_joint_name, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
          break;
        }
        case KEYCODE_Z:
        {
          ROS_DEBUG("Torso Height - Down");
          operateArm(arm_lift_joint_name, 0.0, std::max<int>((int)(std::abs(0.0 - arm_lift_joint_pos1_) / 0.05), 1));
          break;
        }
        case KEYCODE_W:
        {
          ROS_DEBUG("Arm Flex Angle - Up");
          operateArm(arm_flex_joint_name, 0.0, std::max((std::abs(0.0 - arm_flex_joint_pos_) * 2), 1.0));
          break;
        }
        case KEYCODE_S:
        {
          ROS_DEBUG("Arm Flex Angle - Stop");
          operateArm(arm_flex_joint_name, arm_flex_joint_pos_, 1.0);
          break;
        }
        case KEYCODE_X:
        {
          ROS_DEBUG("Arm Flex Angle - Down");
          operateArm(arm_flex_joint_name, -2.62, std::max((std::abs(-2.62 - arm_flex_joint_pos_) * 2), 1.0));
          break;
        }
        case KEYCODE_E:
        {
          ROS_DEBUG("Arm Wrist Angle - Up");
          operateArm(wrist_flex_joint_name, 1.22, std::max((std::abs(1.22 - wrist_flex_joint_pos_) * 2), 1.0));
          break;
        }
        case KEYCODE_D:
        {
          ROS_DEBUG("Arm Wrist Angle - Stop");
          operateArm(wrist_flex_joint_name, wrist_flex_joint_pos_, 1.0);
          break;
        }
        case KEYCODE_C:
        {
          ROS_DEBUG("Arm Wrist Angle - Down");
          operateArm(wrist_flex_joint_name, -1.92, std::max((std::abs(-1.92 - wrist_flex_joint_pos_) * 2), 1.0));
          break;
        }
        case KEYCODE_G:
        {
          moveHand(is_hand_open);

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
  ros::init(argc, argv, "hsr_teleop_key");
  SIGVerseHsrTeleopKey hsr_teleop_key;
  return hsr_teleop_key.run();
}
