#include <cstdio>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <functional>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class SIGVerseTb3OpenManipulatorTeleopKey
{
private:
  static const char KEY_1 = 0x31;
  static const char KEY_2 = 0x32;
  static const char KEY_3 = 0x33;
  static const char KEY_4 = 0x34;
  static const char KEY_5 = 0x35;
  static const char KEY_6 = 0x36;
  static const char KEY_7 = 0x37;
  static const char KEY_8 = 0x38;

  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;

  static const char KEY_A = 0x61;
  static const char KEY_C = 0x63;
  static const char KEY_D = 0x64;
  static const char KEY_H = 0x68;
  static const char KEY_J = 0x6a;
  static const char KEY_M = 0x6d;
  static const char KEY_O = 0x6f;
  static const char KEY_S = 0x73;
  static const char KEY_U = 0x75;
  static const char KEY_W = 0x77;
  
  static const char KEYCODE_SPACE  = 0x20;


  const std::string JOINT1_NAME = "joint1";
  const std::string JOINT2_NAME = "joint2";
  const std::string JOINT3_NAME = "joint3";
  const std::string JOINT4_NAME = "joint4";

  const std::string GRIP_JOINT_NAME     = "grip_joint";
  const std::string GRIP_JOINT_SUB_NAME = "grip_joint_sub";

  const double LINEAR_VEL  = 0.2;
  const double ANGULAR_VEL = 0.4;
  const double JOINT_MIN = -2.83;
  const double JOINT_MAX = +2.83;
  const double GRIP_MIN = -0.01;
  const double GRIP_MAX = +0.035;

public:
  SIGVerseTb3OpenManipulatorTeleopKey();

  void keyLoop(int argc, char** argv);

private:

  static void rosSigintHandler(int sig);
  static int  canReceiveKey( const int fd );

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  void moveBase(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, const double linear_x, const double angular_z);
  void moveArm(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, const std::string &name, const double position, const double current_pos);
  void moveHand(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, const double position, const double current_pos);
  void stopJoints(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, const int duration_sec);

  static int calcTrajectoryDuration(const double val, const double current_val);

  void showHelp();

  rclcpp::Node::SharedPtr node_;

  // Current positions updated by JointState
  double joint1_pos1_, joint2_pos1_, joint3_pos1_, joint4_pos1_, grip_joint_pos1_;
  double joint1_pos2_, joint2_pos2_, joint3_pos2_, joint4_pos2_;
};


SIGVerseTb3OpenManipulatorTeleopKey::SIGVerseTb3OpenManipulatorTeleopKey()
{
  joint1_pos1_ = 0.0; joint2_pos1_ = 0.0; joint3_pos1_ = 0.0; joint4_pos1_ = 0.0; grip_joint_pos1_ = 0.0;
  joint1_pos2_ = 0.0; joint2_pos2_ = 0.0; joint3_pos2_ = 0.0; joint4_pos2_ = 0.0;
}


void SIGVerseTb3OpenManipulatorTeleopKey::rosSigintHandler([[maybe_unused]] int sig)
{
  rclcpp::shutdown();
}


int SIGVerseTb3OpenManipulatorTeleopKey::canReceiveKey( const int fd )
{
  fd_set fdset;
  struct timeval timeout;
  FD_ZERO( &fdset );
  FD_SET( fd , &fdset );

  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  return select( fd+1 , &fdset , NULL , NULL , &timeout );
}

void SIGVerseTb3OpenManipulatorTeleopKey::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
  for(size_t i=0; i<joint_state->name.size(); i++)
  {
    if(joint_state->name[i]==JOINT1_NAME)
    {
      joint1_pos2_ = joint1_pos1_;
      joint1_pos1_ = joint_state->position[i];
    }
    else if(joint_state->name[i]==JOINT2_NAME)
    {
      joint2_pos2_ = joint2_pos1_;
      joint2_pos1_ = joint_state->position[i];
    }
    else if(joint_state->name[i]==JOINT3_NAME)
    {
      joint3_pos2_ = joint3_pos1_;
      joint3_pos1_ = joint_state->position[i];
    }
    else if(joint_state->name[i]==JOINT4_NAME)
    {
      joint4_pos2_ = joint4_pos1_;
      joint4_pos1_ = joint_state->position[i];
    }
    else if(joint_state->name[i]==GRIP_JOINT_NAME)
    {
      grip_joint_pos1_ = joint_state->position[i];
    }
  }
}

void SIGVerseTb3OpenManipulatorTeleopKey::moveBase(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher, const double linear_x, const double angular_z)
{
  geometry_msgs::msg::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = 0.0;
  twist.linear.z  = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_z;

  publisher->publish(twist);
}


void SIGVerseTb3OpenManipulatorTeleopKey::moveArm(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, const std::string &name, const double position, const double current_pos)
{
  std::vector<std::string> names;
  names.push_back(name);

  std::vector<double> positions;
  positions.push_back(position);

  double duration_sec = calcTrajectoryDuration(position, current_pos);

  builtin_interfaces::msg::Duration duration;
  duration.sec  = static_cast<int32_t>(duration_sec);
  duration.nanosec = static_cast<uint32_t>((duration_sec - duration.sec) * 1e9);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;

  trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

  joint_trajectory.points.push_back(arm_joint_point);

  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = duration;

  publisher->publish(joint_trajectory);
}

void SIGVerseTb3OpenManipulatorTeleopKey::moveHand(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, const double position, const double current_pos)
{
  std::vector<std::string> joint_names {GRIP_JOINT_NAME, GRIP_JOINT_SUB_NAME};

  std::vector<double> positions;

  positions.push_back(position); // for grip_joint
  positions.push_back(position); // for grip_joint_sub

  double duration_sec = calcTrajectoryDuration(position, current_pos);

  builtin_interfaces::msg::Duration duration;
  duration.sec  = static_cast<int32_t>(duration_sec);
  duration.nanosec = static_cast<uint32_t>((duration_sec - duration.sec) * 1e9);

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = duration;

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);

  publisher->publish(joint_trajectory);
}


void SIGVerseTb3OpenManipulatorTeleopKey::stopJoints(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, const int duration_sec)
{
  std::vector<std::string> names {JOINT1_NAME, JOINT2_NAME, JOINT3_NAME, JOINT4_NAME};

  std::vector<double> positions;
  positions.push_back(2.0*joint1_pos1_-joint1_pos2_);
  positions.push_back(2.0*joint2_pos1_-joint2_pos2_);
  positions.push_back(2.0*joint3_pos1_-joint3_pos2_);
  positions.push_back(2.0*joint4_pos1_-joint4_pos2_);

  builtin_interfaces::msg::Duration duration;
  duration.sec = duration_sec;
  duration.nanosec = 0;

  trajectory_msgs::msg::JointTrajectory joint_trajectory;

  trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

  joint_trajectory.points.push_back(arm_joint_point);

  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = duration;

  publisher->publish(joint_trajectory);
}


int SIGVerseTb3OpenManipulatorTeleopKey::calcTrajectoryDuration(const double val, const double current_val)
{
  return std::max<int>((int)(std::abs(val - current_val) / 0.5), 1);
}


void SIGVerseTb3OpenManipulatorTeleopKey::showHelp()
{
  puts("\n");
  puts("---------------------------");
  puts("Operate from keyboard");
  puts("---------------------------");
  puts("arrow keys : Move");
  puts("---------------------------");
  puts("Space: Stop");
  puts("---------------------------");
  puts("w: Go Forward");
  puts("s: Go Back");
  puts("d: Turn Right");
  puts("a: Turn Left");
  puts("---------------------------");
  puts("u: Rotate Arm - Upward");
  puts("j: Rotate Arm - Horizontal");
  puts("m: Rotate Arm - Downward");
  puts("---------------------------");
  puts("1/2: Joint1 Right/Left");
  puts("3/4: Joint2 Up/Down");
  puts("5/6: Joint3 Up/Down");
  puts("7/8: Joint4 Up/Down");
  puts("---------------------------");
  puts("o: Hand Open");
  puts("c: Hand Close");
  puts("---------------------------");
  puts("h: Show help");
}


void SIGVerseTb3OpenManipulatorTeleopKey::keyLoop(int argc, char** argv)
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

  rclcpp::init(argc, argv);

  node_ = rclcpp::Node::make_shared("tb3_omc_teleop_key");

  auto logger = node_->get_logger();

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  rclcpp::Rate loop_rate(10);

  auto sub_joint_state = node_->create_subscription<sensor_msgs::msg::JointState>("/tb3omc/joint_state", 10, std::bind(&SIGVerseTb3OpenManipulatorTeleopKey::jointStateCallback, this, std::placeholders::_1));
  auto pub_base_twist  = node_->create_publisher<geometry_msgs::msg::Twist>("/tb3omc/cmd_vel", 10);
  auto pub_joint_traj  = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/tb3omc/joint_trajectory", 10);

  sleep(2);

  showHelp();

  while (rclcpp::ok())
  {
    if(canReceiveKey(kfd))
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
        case KEYCODE_SPACE:
        {
          RCLCPP_DEBUG(logger, "Stop");
          moveBase(pub_base_twist, 0.0, 0.0);
          stopJoints(pub_joint_traj, 1.0);
          break;
        }
        case KEY_W:
        case KEYCODE_UP:
        {
          RCLCPP_DEBUG(logger, "Go Forward");
          moveBase(pub_base_twist, +LINEAR_VEL, 0.0);
          break;
        }
        case KEY_S:
        case KEYCODE_DOWN:
        {
          RCLCPP_DEBUG(logger, "Go Back");
          moveBase(pub_base_twist, -LINEAR_VEL, 0.0);
          break;
        }
        case KEY_D:
        case KEYCODE_RIGHT:
        {
          RCLCPP_DEBUG(logger, "Turn Right");
          moveBase(pub_base_twist, 0.0, -ANGULAR_VEL);
          break;
        }
        case KEY_A:
        case KEYCODE_LEFT:
        {
          RCLCPP_DEBUG(logger, "Turn Left");
          moveBase(pub_base_twist, 0.0, +ANGULAR_VEL);
          break;
        }
        case KEY_U:
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Upward");
          moveArm(pub_joint_traj, JOINT2_NAME, 0.0, joint2_pos1_);
          moveArm(pub_joint_traj, JOINT3_NAME, 0.0, joint3_pos1_);
          moveArm(pub_joint_traj, JOINT4_NAME, 0.0, joint4_pos1_);
          break;
        }
        case KEY_J:
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Horizontal");
          moveArm(pub_joint_traj, JOINT2_NAME, +1.57, joint2_pos1_);
          moveArm(pub_joint_traj, JOINT3_NAME, -1.57, joint3_pos1_);
          moveArm(pub_joint_traj, JOINT4_NAME, +0.26, joint4_pos1_);
          break;
        }
        case KEY_M:
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Downward");
          moveArm(pub_joint_traj, JOINT2_NAME, +1.75, joint2_pos1_);
          moveArm(pub_joint_traj, JOINT3_NAME, -1.57, joint3_pos1_);
          moveArm(pub_joint_traj, JOINT4_NAME, +0.26, joint4_pos1_);
          break;
        }
        case KEY_1:
        {
          RCLCPP_DEBUG(logger, "Joint1 Right");
          moveArm(pub_joint_traj, JOINT1_NAME, JOINT_MIN, joint1_pos1_);
          break;
        }
        case KEY_2:
        {
          RCLCPP_DEBUG(logger, "Joint1 Left");
          moveArm(pub_joint_traj, JOINT1_NAME, JOINT_MAX, joint1_pos1_);
          break;
        }
        case KEY_3:
        {
          RCLCPP_DEBUG(logger, "Joint2 Up");
          moveArm(pub_joint_traj, JOINT2_NAME, JOINT_MIN, joint2_pos1_);
          break;
        }
        case KEY_4:
        {
          RCLCPP_DEBUG(logger, "Joint2 Down");
          moveArm(pub_joint_traj, JOINT2_NAME, JOINT_MAX, joint2_pos1_);
          break;
        }
        case KEY_5:
        {
          RCLCPP_DEBUG(logger, "Joint3 Up");
          moveArm(pub_joint_traj, JOINT3_NAME, JOINT_MIN, joint3_pos1_);
          break;
        }
        case KEY_6:
        {
          RCLCPP_DEBUG(logger, "Joint3 Down");
          moveArm(pub_joint_traj, JOINT3_NAME, JOINT_MAX, joint3_pos1_);
          break;
        }
        case KEY_7:
        {
          RCLCPP_DEBUG(logger, "Joint4 Up");
          moveArm(pub_joint_traj, JOINT4_NAME, JOINT_MIN, joint4_pos1_);
          break;
        }
        case KEY_8:
        {
          RCLCPP_DEBUG(logger, "Joint4 Down");
          moveArm(pub_joint_traj, JOINT4_NAME, JOINT_MAX, joint4_pos1_);
          break;
        }
        case KEY_O:
        {
          RCLCPP_DEBUG(logger, "Hand Open");
          moveHand(pub_joint_traj, GRIP_MIN, grip_joint_pos1_);
          break;
        }
        case KEY_C:
        {
          RCLCPP_DEBUG(logger, "Hand Close");
          moveHand(pub_joint_traj, GRIP_MAX, grip_joint_pos1_);
          break;
        }
        case KEY_H:
        {
          RCLCPP_DEBUG(logger, "Show Help");
          showHelp();
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

  return;
}


int main(int argc, char** argv)
{
  SIGVerseTb3OpenManipulatorTeleopKey teleop_key;

  teleop_key.keyLoop(argc, argv);

  return(EXIT_SUCCESS);
}

