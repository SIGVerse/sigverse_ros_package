#include <stdio.h>
#include <cmath>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tmc_suction/suction_server.hpp>

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
  static const char KEYCODE_E = 0x65;
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
  static const char KEYCODE_U = 0x75;
  static const char KEYCODE_V = 0x76;
  static const char KEYCODE_W = 0x77;
  static const char KEYCODE_Y = 0x79;
  static const char KEYCODE_Z = 0x7a;

  static const char KEYCODE_COMMA  = 0x2c;
  static const char KEYCODE_PERIOD = 0x2e;
  static const char KEYCODE_SPACE  = 0x20;

  const std::string MSG_TELL_ME  = "Please tell me";
  const std::string MSG_POINT_IT = "Please point it";

  const std::string ARM_LIFT_JOINT_NAME   = "arm_lift_joint";
  const std::string ARM_FLEX_JOINT_NAME   = "arm_flex_joint";
  const std::string ARM_ROLL_JOINT_NAME   = "arm_roll_joint";
  const std::string WRIST_FLEX_JOINT_NAME = "wrist_flex_joint";
  const std::string WRIST_ROLL_JOINT_NAME = "wrist_roll_joint";

public:
  SIGVerseHsrTeleopKey();

  static void rosSigintHandler(int sig);
  static int  canReceive(int fd);

  void messageCallback(const std_msgs::String::ConstPtr& message);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  void sendMessage(const std::string &message);
  void moveBaseTwist(const double linear_x, const double linear_y, const double angular_z);
  void moveBaseJointTrajectory(const double linear_x, const double linear_y, const double theta, const double duration_sec);
  void operateArm(const double arm_lift_pos, const double arm_flex_pos, const double arm_roll_pos, const double wrist_flex_pos, const double wrist_roll_pos, const double duration_sec);
  void operateArm(const std::string &name, const double position, const double duration_sec);
  void operateArmFlex(const double arm_flex_pos, const double wrist_flex_pos);
  double getDurationRot(const double next_pos, const double current_pos);
  void operateHand(const bool is_hand_open);
  void sendSuctionGoal(const bool &sution_on);
  void suctionResultCallback(const tmc_suction::SuctionControlActionResult::ConstPtr& suction_result);

  void showHelp();
  int run(int argc, char **argv);

private:
  // Last position and previous position of arm_lift_joint
  double arm_lift_joint_pos1_;
  double arm_lift_joint_pos2_;
  double arm_flex_joint_pos_;
  double arm_roll_joint_pos_;
  double wrist_flex_joint_pos_;
  double wrist_roll_joint_pos_;

  ros::NodeHandle node_handle_;

  ros::Subscriber sub_msg_;
  ros::Publisher  pub_msg_;
  ros::Subscriber sub_joint_state_;
  ros::Publisher  pub_base_twist_;
  ros::Publisher  pub_base_trajectory_;
  ros::Publisher  pub_arm_trajectory_;
  ros::Publisher  pub_gripper_trajectory_;
  ros::Publisher  pub_suction_goal_;
  ros::Subscriber sub_suction_result_; 

  tf::TransformListener listener_;
};


SIGVerseHsrTeleopKey::SIGVerseHsrTeleopKey()
{
  arm_lift_joint_pos1_   = 0.0;
  arm_lift_joint_pos2_   = 0.0;
  arm_flex_joint_pos_    = 0.0;
  arm_roll_joint_pos_    = 0.0;
  wrist_flex_joint_pos_  = 0.0;
  wrist_roll_joint_pos_  = 0.0;
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
    if(joint_state->name[i] == ARM_LIFT_JOINT_NAME)
    {
      arm_lift_joint_pos2_ = arm_lift_joint_pos1_;
      arm_lift_joint_pos1_ = joint_state->position[i];
    }
    if(joint_state->name[i] == ARM_FLEX_JOINT_NAME)
    {
      arm_flex_joint_pos_ = joint_state->position[i];
    }
    if(joint_state->name[i] == ARM_ROLL_JOINT_NAME)
    {
      arm_roll_joint_pos_ = joint_state->position[i];
    }
    if(joint_state->name[i] == WRIST_FLEX_JOINT_NAME)
    {
      wrist_flex_joint_pos_ = joint_state->position[i];
    }
    if(joint_state->name[i] == WRIST_ROLL_JOINT_NAME)
    {
      wrist_roll_joint_pos_ = joint_state->position[i];
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

void SIGVerseHsrTeleopKey::moveBaseTwist(const double linear_x, const double linear_y, const double angular_z)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_.publish(twist);
}

void SIGVerseHsrTeleopKey::moveBaseJointTrajectory(const double linear_x, const double linear_y, const double theta, const double duration_sec)
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

void SIGVerseHsrTeleopKey::operateArm(const double arm_lift_pos, const double arm_flex_pos, const double arm_roll_pos, const double wrist_flex_pos, const double wrist_roll_pos, const double duration_sec)
{
  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back(ARM_LIFT_JOINT_NAME);
  joint_trajectory.joint_names.push_back(ARM_FLEX_JOINT_NAME);
  joint_trajectory.joint_names.push_back(ARM_ROLL_JOINT_NAME);
  joint_trajectory.joint_names.push_back(WRIST_FLEX_JOINT_NAME);
  joint_trajectory.joint_names.push_back(WRIST_ROLL_JOINT_NAME);

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;

  arm_joint_point.positions = {arm_lift_pos, arm_flex_pos, arm_roll_pos, wrist_flex_pos, wrist_roll_pos};

  arm_joint_point.time_from_start = ros::Duration(duration_sec);
  joint_trajectory.points.push_back(arm_joint_point);
  pub_arm_trajectory_.publish(joint_trajectory);
}

void SIGVerseHsrTeleopKey::operateArm(const std::string &name, const double position, const double duration_sec)
{
  double arm_lift = arm_lift_joint_pos1_;
  double arm_flex = arm_flex_joint_pos_;
  double arm_roll = arm_roll_joint_pos_;
  double wrist_flex = wrist_flex_joint_pos_;
  double wrist_roll = wrist_roll_joint_pos_;

  if     (name == ARM_LIFT_JOINT_NAME)  { arm_lift   = position; }
  else if(name == ARM_FLEX_JOINT_NAME)  { arm_flex   = position; }
  else if(name == ARM_ROLL_JOINT_NAME)  { arm_roll   = position; }
  else if(name == WRIST_FLEX_JOINT_NAME){ wrist_flex = position; }
  else if(name == WRIST_ROLL_JOINT_NAME){ wrist_roll = position; }

  this->operateArm(arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll, duration_sec);
}

void SIGVerseHsrTeleopKey::operateArmFlex(const double arm_flex_pos, const double wrist_flex_pos)
{
  double duration = std::max(this->getDurationRot(arm_flex_pos, arm_flex_joint_pos_), this->getDurationRot(wrist_flex_pos, wrist_flex_joint_pos_));

  this->operateArm(arm_lift_joint_pos1_, arm_flex_pos, arm_roll_joint_pos_, wrist_flex_pos, wrist_roll_joint_pos_, duration);
}

double SIGVerseHsrTeleopKey::getDurationRot(const double next_pos, const double current_pos)
{
  return std::max<double>((std::abs(next_pos - current_pos) * 1.2), 1.0);
}

void SIGVerseHsrTeleopKey::operateHand(const bool is_hand_open)
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

void SIGVerseHsrTeleopKey::sendSuctionGoal(const bool &sution_on)
{
  tmc_suction::SuctionControlActionGoal goal_msg;
  goal_msg.goal.timeout = ros::Duration(10.0);
  goal_msg.goal.suction_on.data = sution_on;
  pub_suction_goal_.publish(goal_msg);
}

void SIGVerseHsrTeleopKey::suctionResultCallback(const tmc_suction::SuctionControlActionResult::ConstPtr& suction_result) 
{
  // 3: SUCCEEDED
  // 4: ABORTED(Timeout)
  ROS_INFO("Subscribe Suction result: status=%i", suction_result->status.status);
}
  
void SIGVerseHsrTeleopKey::showHelp()
{
  puts("Operate by Keyboard");
  puts("---------------------------");
  puts("arrow keys : Move HSR");
  puts("space      : Stop HSR");
  puts("---------------------------");
  puts("Move HSR Linearly (1m)");
  puts("  u   i   o  ");
  puts("  j   k   l  ");
  puts("  m   ,   .  ");
  puts("---------------------------");
  puts("q/z : Increase/Decrease Moving Speed");
  puts("---------------------------");
  puts("y : Up   Torso");
  puts("h : Stop Torso");
  puts("n : Down Torso");
  puts("---------------------------");
  puts("a : Rotate Arm - Vertical");
  puts("b : Rotate Arm - Upward");
  puts("c : Rotate Arm - Horizontal");
  puts("d : Rotate Arm - Downward");
  puts("e : Rotate Arm - Suction Downward");
  puts("---------------------------");
  puts("g : Grasp/Open Hand");
  puts("v : Suction ON");
  puts("w : Suction OFF");
  puts("---------------------------");
  puts(("1 : Send Message: "+MSG_TELL_ME).c_str());
  puts(("2 : Send Message: "+MSG_POINT_IT).c_str());
}


int SIGVerseHsrTeleopKey::run(int argc, char **argv)
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

  sub_msg_                = node_handle_.subscribe<std_msgs::String>                ("/hsrb/message/to_robot", 100, &SIGVerseHsrTeleopKey::messageCallback, this);
  pub_msg_                = node_handle_.advertise<std_msgs::String>                ("/hsrb/message/to_human", 10);
  sub_joint_state_        = node_handle_.subscribe<sensor_msgs::JointState>         ("/hsrb/joint_states", 10, &SIGVerseHsrTeleopKey::jointStateCallback, this);
  pub_base_twist_         = node_handle_.advertise<geometry_msgs::Twist>            ("/hsrb/command_velocity", 10);
  pub_base_trajectory_    = node_handle_.advertise<trajectory_msgs::JointTrajectory>("/hsrb/omni_base_controller/command", 10);
  pub_arm_trajectory_     = node_handle_.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
  pub_gripper_trajectory_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>("/hsrb/gripper_controller/command", 10);
  pub_suction_goal_       = node_handle_.advertise<tmc_suction::SuctionControlActionGoal>  ("/hsrb/suction_control/goal", 10, false);
  sub_suction_result_     = node_handle_.subscribe<tmc_suction::SuctionControlActionResult>("/hsrb/suction_control/result", 10, &SIGVerseHsrTeleopKey::suctionResultCallback, this);

  const float linear_coef  = 0.2f;
  const float angular_coef = 0.5f;

  float move_speed = 1.0f;
  bool is_hand_open = false;
  bool is_suction_on = false;

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
          ROS_DEBUG("Move Left Forward");
          moveBaseJointTrajectory(+1.0, +1.0, +M_PI_4, 10);
          break;
        }
        case KEYCODE_I:
        {
          ROS_DEBUG("Move Forward");
          moveBaseJointTrajectory(+1.0, 0.0, 0.0, 10);
          break;
        }
        case KEYCODE_O:
        {
          ROS_DEBUG("Move Right Forward");
          moveBaseJointTrajectory(+1.0, -1.0, -M_PI_4, 10);
          break;
        }
        case KEYCODE_J:
        {
          ROS_DEBUG("Move Left");
          moveBaseJointTrajectory(0.0, +1.0, +M_PI_2, 10);
          break;
        }
        case KEYCODE_K:
        {
          ROS_DEBUG("Stop");
          moveBaseJointTrajectory(0.0, 0.0, 0.0, 0.5);
          break;
        }
        case KEYCODE_L:
        {
          ROS_DEBUG("Move Right");
          moveBaseJointTrajectory(0.0, -1.0, -M_PI_2, 10);
          break;
        }
        case KEYCODE_M:
        {
          ROS_DEBUG("Move Left Backward");
          moveBaseJointTrajectory(-1.0, +1.0, +M_PI_2+M_PI_4, 10);
          break;
        }
        case KEYCODE_COMMA:
        {
          ROS_DEBUG("Move Backward");
          moveBaseJointTrajectory(-1.0, 0.0, +M_PI, 10);
          break;
        }
        case KEYCODE_PERIOD:
        {
          ROS_DEBUG("Move Right Backward");
          moveBaseJointTrajectory(-1.0, -1.0, -M_PI_2-M_PI_4, 10);
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
          ROS_DEBUG("Up Torso");
          operateArm(ARM_LIFT_JOINT_NAME, 0.69, std::max<int>((int)(std::abs(0.69 - arm_lift_joint_pos1_) / 0.05), 1)/move_speed);
          break;
        }
        case KEYCODE_H:
        {
          ROS_DEBUG("Stop Torso");
          operateArm(ARM_LIFT_JOINT_NAME, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
          break;
        }
        case KEYCODE_N:
        {
          ROS_DEBUG("Down Torso");
          operateArm(ARM_LIFT_JOINT_NAME, 0.0, std::max<int>((int)(std::abs(0.0 - arm_lift_joint_pos1_) / 0.05), 1)/move_speed);
          break;
        }
        //operateArm(const double arm_lift_pos, const double arm_flex_pos, const double wrist_flex_pos, const int duration_sec);
        case KEYCODE_A:
        {
          ROS_DEBUG("Rotate Arm - Vertical");
          operateArmFlex(0.0, -1.57);
          break;
        }
        case KEYCODE_B:
        {
          ROS_DEBUG("Rotate Arm - Upward");
          operateArmFlex(-0.785, -0.785);
          break;
        }
        case KEYCODE_C:
        {
          ROS_DEBUG("Rotate Arm - Horizontal");
          operateArmFlex(-1.57, 0.0);
          break;
        }
        case KEYCODE_D:
        {
          ROS_DEBUG("Rotate Arm - Downward");
          operateArmFlex(-2.2, 0.35);
          break;
        }
        case KEYCODE_E:
        {
          ROS_DEBUG("Rotate Arm - Suction Downward");
          operateArm(arm_lift_joint_pos1_, -2.01, -0.878, -1.108, 0.174, 3);
          break;
        }
        case KEYCODE_G:
        {
          operateHand(is_hand_open);

          is_hand_open = !is_hand_open;
          break;
        }
        case KEYCODE_V:
        {
		  sendSuctionGoal(true);
          break;
        }
        case KEYCODE_W:
        {
		  sendSuctionGoal(false);
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
  return hsr_teleop_key.run(argc, argv);
}
