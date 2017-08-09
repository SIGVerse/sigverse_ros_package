#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class SIGVerseTb3OmcTeleopKey
{
private:
  static const char KEY_A = 0x61;
  static const char KEY_C = 0x63;
  static const char KEY_D = 0x64;
  static const char KEY_G = 0x67;
  static const char KEY_H = 0x68;
  static const char KEY_I = 0x69;
  static const char KEY_J = 0x6a;
  static const char KEY_K = 0x6b;
  static const char KEY_O = 0x6f;
  static const char KEY_Q = 0x71;
  static const char KEY_S = 0x73;
  static const char KEY_T = 0x74;
  static const char KEY_U = 0x75;
  static const char KEY_W = 0x77;
  static const char KEY_X = 0x78;
  static const char KEY_Y = 0x79;

  const std::string JOINT1_NAME = "joint1";
  const std::string JOINT2_NAME = "joint2";
  const std::string JOINT3_NAME = "joint3";
  const std::string JOINT4_NAME = "joint4";

  const std::string GRIP_JOINT_NAME     = "grip_joint";
  const std::string GRIP_JOINT_SUB_NAME = "grip_joint_sub";

  const double JOINT_MIN = -2.83;
  const double JOINT_MAX = +2.83;
  const double GRIP_MIN = -0.01;
  const double GRIP_MAX = +0.04;

public:
  SIGVerseTb3OmcTeleopKey();

  static void rosSigintHandler(int sig);
  int  canReceive( const int fd );

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  void moveBase(ros::Publisher &publisher, const double linear_x, const double angular_z);
  void moveArm(ros::Publisher &publisher, const std::string &name, const double position, const double current_pos);
  void moveHand(ros::Publisher &publisher, const double position, const double current_pos);
  void stopJoints(ros::Publisher &publisher, const int duration_sec);

  static int calcDuration(const double val, const double current_val);

  void showHelp();
  void keyLoop(int argc, char** argv);

private:

  // Current positions that is updated by JointState
  double joint1_pos_, joint2_pos_, joint3_pos_, joint4_pos_, grip_joint_pos_;
};


SIGVerseTb3OmcTeleopKey::SIGVerseTb3OmcTeleopKey()
{
  joint1_pos_ = 0.0;
  joint2_pos_ = 0.0;
  joint3_pos_ = 0.0;
  joint4_pos_ = 0.0;
  grip_joint_pos_ = 0.0;
}


void SIGVerseTb3OmcTeleopKey::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int SIGVerseTb3OmcTeleopKey::canReceive( const int fd )
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

void SIGVerseTb3OmcTeleopKey::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
//  ROS_INFO("jointStateCallback size=%d", (int)joint_state->name.size());

  for(int i=0; i<joint_state->name.size(); i++)
  {
    if(joint_state->name[i]==JOINT1_NAME)
    {
      joint1_pos_ = joint_state->position[i];
      continue;
    }
    if(joint_state->name[i]==JOINT2_NAME)
    {
      joint2_pos_ = joint_state->position[i];
      continue;
    }
    if(joint_state->name[i]==JOINT3_NAME)
    {
      joint3_pos_ = joint_state->position[i];
      continue;
    }
    if(joint_state->name[i]==JOINT4_NAME)
    {
      joint4_pos_ = joint_state->position[i];
      continue;
    }
    if(joint_state->name[i]==GRIP_JOINT_NAME)
    {
      grip_joint_pos_ = joint_state->position[i];
      continue;
    }
  }
}

void SIGVerseTb3OmcTeleopKey::moveBase(ros::Publisher &publisher, const double linear_x, const double angular_z)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = 0.0;
  twist.linear.z  = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_z;

  publisher.publish(twist);
}


void SIGVerseTb3OmcTeleopKey::moveArm(ros::Publisher &publisher, const std::string &name, const double position, const double current_pos)
{
  std::vector<std::string> names;
  names.push_back(name);

  std::vector<double> positions;
  positions.push_back(position);

  ros::Duration duration;
  duration.sec = calcDuration(position, current_pos);

  trajectory_msgs::JointTrajectory joint_trajectory;

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;

  joint_trajectory.points.push_back(arm_joint_point);

  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = duration;

  publisher.publish(joint_trajectory);
}

void SIGVerseTb3OmcTeleopKey::moveHand(ros::Publisher &publisher, const double position, const double current_pos)
{
  std::vector<std::string> joint_names {GRIP_JOINT_NAME, GRIP_JOINT_SUB_NAME};

  std::vector<double> positions;

  positions.push_back(position); // for grip_joint
  positions.push_back(position); // for grip_joint_sub

  ros::Duration duration;
  duration.sec = calcDuration(position, current_pos);

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = duration;

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);

  publisher.publish(joint_trajectory);
}


void SIGVerseTb3OmcTeleopKey::stopJoints(ros::Publisher &publisher, const int duration_sec)
{
//  std::vector<std::string> names {JOINT1_NAME, JOINT2_NAME, JOINT3_NAME, JOINT4_NAME, GRIP_JOINT_NAME, GRIP_JOINT_SUB_NAME};
  std::vector<std::string> names {JOINT1_NAME, JOINT2_NAME, JOINT3_NAME, JOINT4_NAME};

  std::vector<double> positions;
  positions.push_back(joint1_pos_);
  positions.push_back(joint2_pos_);
  positions.push_back(joint3_pos_);
  positions.push_back(joint4_pos_);
//  positions.push_back(grip_joint_pos_);
//  positions.push_back(grip_joint_pos_); // grip_joint_sub is same as grip_joint

  ros::Duration duration;
  duration.sec = duration_sec;

  trajectory_msgs::JointTrajectory joint_trajectory;

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;

  joint_trajectory.points.push_back(arm_joint_point);

  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = duration;

  publisher.publish(joint_trajectory);
}


int SIGVerseTb3OmcTeleopKey::calcDuration(const double val, const double current_val)
{
  return std::max<int>((int)(std::abs(val - current_val) / 0.5), 1);
}


void SIGVerseTb3OmcTeleopKey::showHelp()
{
  puts("Operate from keyboard");
  puts("---------------------------");
  puts("s : Stop");
  puts("---------------------------");
  puts("w : Go Forward");
  puts("x : Go Back");
  puts("d : Turn Right");
  puts("a : Turn Left");
  puts("---------------------------");
  puts("t : Joint1 Right");
  puts("g : Joint1 Left");
  puts("y : Joint2 Up");
  puts("h : Joint2 Down");
  puts("u : Joint3 Up");
  puts("j : Joint3 Down");
  puts("i : Joint4 Up");
  puts("k : Joint4 Down");
  puts("---------------------------");
  puts("o : Hand Open");
  puts("c : Hand Close");
//  puts("---------------------------");
//  puts("h : Show help");
}


void SIGVerseTb3OmcTeleopKey::keyLoop(int argc, char** argv)
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

  ros::init(argc, argv, "tb3_omc_teleop_key", ros::init_options::NoSigintHandler);

  ros::NodeHandle node_handle;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(30);

  std::string sub_joint_state_topic_name      = "/tb3omc/joint_state";
  std::string pub_base_twist_topic_name       = "/tb3omc/cmd_vel";
  std::string pub_joint_trajectory_topic_name = "/tb3omc/joint_trajectory";


  ros::Subscriber sub_joint_state = node_handle.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &SIGVerseTb3OmcTeleopKey::jointStateCallback, this);

  ros::Publisher pub_base_twist = node_handle.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
  ros::Publisher pub_joint_traj = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_joint_trajectory_topic_name, 10);

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
        case KEY_S:
        {
          ROS_DEBUG("Stop");
          moveBase(pub_base_twist, 0.0, 0.0);
          stopJoints(pub_joint_traj, 1);
          break;
        }
        case KEY_W:
        {
          ROS_DEBUG("Go Forward");
          moveBase(pub_base_twist, +1.0, 0.0);
          break;
        }
        case KEY_X:
        {
          ROS_DEBUG("Go Back");
          moveBase(pub_base_twist, -1.0, 0.0);
          break;
        }
        case KEY_D:
        {
          ROS_DEBUG("Turn Right");
          moveBase(pub_base_twist, 0.0, -1.0);
          break;
        }
        case KEY_A:
        {
          ROS_DEBUG("Turn Left");
          moveBase(pub_base_twist, 0.0, +1.0);
          break;
        }
        case KEY_T:
        {
          ROS_DEBUG("Joint1 Right");
          moveArm(pub_joint_traj, JOINT1_NAME, JOINT_MIN, joint1_pos_);
          break;
        }
        case KEY_G:
        {
          ROS_DEBUG("Joint1 Left");
          moveArm(pub_joint_traj, JOINT1_NAME, JOINT_MAX, joint1_pos_);
          break;
        }
        case KEY_Y:
        {
          ROS_DEBUG("Joint2 Up");
          moveArm(pub_joint_traj, JOINT2_NAME, JOINT_MIN, joint2_pos_);
          break;
        }
        case KEY_H:
        {
          ROS_DEBUG("Joint2 Down");
          moveArm(pub_joint_traj, JOINT2_NAME, JOINT_MAX, joint2_pos_);
          break;
        }
        case KEY_U:
        {
          ROS_DEBUG("Joint3 Up");
          moveArm(pub_joint_traj, JOINT3_NAME, JOINT_MIN, joint3_pos_);
          break;
        }
        case KEY_J:
        {
          ROS_DEBUG("Joint3 Down");
          moveArm(pub_joint_traj, JOINT3_NAME, JOINT_MAX, joint3_pos_);
          break;
        }
        case KEY_I:
        {
          ROS_DEBUG("Joint4 Up");
          moveArm(pub_joint_traj, JOINT4_NAME, JOINT_MIN, joint4_pos_);
          break;
        }
        case KEY_K:
        {
          ROS_DEBUG("Joint4 Down");
          moveArm(pub_joint_traj, JOINT4_NAME, JOINT_MAX, joint4_pos_);
          break;
        }
        case KEY_O:
        {
          ROS_DEBUG("Hand Open");
          moveHand(pub_joint_traj, GRIP_MIN, grip_joint_pos_);
          break;
        }
        case KEY_C:
        {
          ROS_DEBUG("Hand Close");
          moveHand(pub_joint_traj, GRIP_MAX, grip_joint_pos_);
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

  return;
}


int main(int argc, char** argv)
{
  SIGVerseTb3OmcTeleopKey tb3_omc_teleop_key;

  tb3_omc_teleop_key.keyLoop(argc, argv);

  return(EXIT_SUCCESS);
}

