#include <cstdio>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class SIGVerseTb3OpenManipulatorGraspingTeleopKey
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

  static const char KEY_A = 0x61;
  static const char KEY_C = 0x63;
  static const char KEY_D = 0x64;
  static const char KEY_H = 0x68;
  static const char KEY_O = 0x6f;
  static const char KEY_S = 0x73;
  static const char KEY_W = 0x77;
  static const char KEY_X = 0x78;

  const std::string JOINT1_NAME = "joint1";
  const std::string JOINT2_NAME = "joint2";
  const std::string JOINT3_NAME = "joint3";
  const std::string JOINT4_NAME = "joint4";

  const std::string GRIP_JOINT_NAME     = "grip_joint";
  const std::string GRIP_JOINT_SUB_NAME = "grip_joint_sub";

  const double LINEAR_VEL  = 0.2;
  const double ANGULAR_VEL = 0.2;
  const double JOINT_MIN = -2.83;
  const double JOINT_MAX = +2.83;
  const double GRIP_MIN = -0.01;
  const double GRIP_MAX = +0.035;

public:
  SIGVerseTb3OpenManipulatorGraspingTeleopKey();

  void keyLoop(int argc, char** argv);

private:

  static void rosSigintHandler(int sig);
  static int  canReceive( const int fd );

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  void moveBase(ros::Publisher &publisher, const double linear_x, const double angular_z);
  void moveArm(ros::Publisher &publisher, const std::string &name, const double position, const double current_pos);
  void moveHand(ros::Publisher &publisher, const double position, const double current_pos);
  void stopJoints(ros::Publisher &publisher, const int duration_sec);

  static int calcTrajectoryDuration(const double val, const double current_val);

  void showHelp();

  // Current positions that is updated by JointState
  double joint1_pos_, joint2_pos_, joint3_pos_, joint4_pos_, grip_joint_pos_;
};


SIGVerseTb3OpenManipulatorGraspingTeleopKey::SIGVerseTb3OpenManipulatorGraspingTeleopKey()
{
  joint1_pos_ = 0.0;
  joint2_pos_ = 0.0;
  joint3_pos_ = 0.0;
  joint4_pos_ = 0.0;
  grip_joint_pos_ = 0.0;
}


void SIGVerseTb3OpenManipulatorGraspingTeleopKey::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int SIGVerseTb3OpenManipulatorGraspingTeleopKey::canReceive( const int fd )
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

void SIGVerseTb3OpenManipulatorGraspingTeleopKey::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
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

void SIGVerseTb3OpenManipulatorGraspingTeleopKey::moveBase(ros::Publisher &publisher, const double linear_x, const double angular_z)
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


void SIGVerseTb3OpenManipulatorGraspingTeleopKey::moveArm(ros::Publisher &publisher, const std::string &name, const double position, const double current_pos)
{
  std::vector<std::string> names;
  names.push_back(name);

  std::vector<double> positions;
  positions.push_back(position);

  ros::Duration duration;
  duration.sec = calcTrajectoryDuration(position, current_pos);

  trajectory_msgs::JointTrajectory joint_trajectory;

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;

  joint_trajectory.points.push_back(arm_joint_point);

  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = duration;

  publisher.publish(joint_trajectory);
}

void SIGVerseTb3OpenManipulatorGraspingTeleopKey::moveHand(ros::Publisher &publisher, const double position, const double current_pos)
{
  std::vector<std::string> joint_names {GRIP_JOINT_NAME, GRIP_JOINT_SUB_NAME};

  std::vector<double> positions;

  positions.push_back(position); // for grip_joint
  positions.push_back(position); // for grip_joint_sub

  ros::Duration duration;
  duration.sec = calcTrajectoryDuration(position, current_pos);

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = duration;

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);

  publisher.publish(joint_trajectory);
}


void SIGVerseTb3OpenManipulatorGraspingTeleopKey::stopJoints(ros::Publisher &publisher, const int duration_sec)
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


int SIGVerseTb3OpenManipulatorGraspingTeleopKey::calcTrajectoryDuration(const double val, const double current_val)
{
  return std::max<int>((int)(std::abs(val - current_val) / 0.5), 1);
}


void SIGVerseTb3OpenManipulatorGraspingTeleopKey::showHelp()
{
  puts("\n");
  puts("---------------------------");
  puts("Operate from keyboard");
  puts("---------------------------");
  puts("s: Stop all movements");
  puts("---------------------------");
  puts("w: Go Forward");
  puts("x: Go Back");
  puts("d: Turn Right");
  puts("a: Turn Left");
  puts("---------------------------");
  puts("1: Joint1 Right");
  puts("2: Joint1 Left");
  puts("3: Joint2 Up");
  puts("4: Joint2 Down");
  puts("5: Joint3 Up");
  puts("6: Joint3 Down");
  puts("7: Joint4 Up");
  puts("8: Joint4 Down");
  puts("---------------------------");
  puts("o: Hand Open");
  puts("c: Hand Close");
  puts("---------------------------");
  puts("h: Show help");
}


void SIGVerseTb3OpenManipulatorGraspingTeleopKey::keyLoop(int argc, char** argv)
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

  ros::init(argc, argv, "tb3_omc_teleop_key", ros::init_options::NoSigintHandler);

  ros::NodeHandle node_handle;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(10);

  std::string sub_joint_state_topic_name;
  std::string pub_base_twist_topic_name;
  std::string pub_joint_trajectory_topic_name;

  node_handle.param<std::string>("sub_joint_state_topic_name",      sub_joint_state_topic_name,      "/tb3omc/joint_state");
  node_handle.param<std::string>("pub_twist_topic_name",            pub_base_twist_topic_name,       "/tb3omc/cmd_vel");
  node_handle.param<std::string>("pub_joint_trajectory_topic_name", pub_joint_trajectory_topic_name, "/tb3omc/joint_trajectory");

  ros::Subscriber sub_joint_state = node_handle.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &SIGVerseTb3OpenManipulatorGraspingTeleopKey::jointStateCallback, this);
  ros::Publisher pub_base_twist = node_handle.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
  ros::Publisher pub_joint_traj = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_joint_trajectory_topic_name, 10);

  sleep(2);

  showHelp();

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
          moveBase(pub_base_twist, +LINEAR_VEL, 0.0);
          break;
        }
        case KEY_X:
        {
          ROS_DEBUG("Go Back");
          moveBase(pub_base_twist, -LINEAR_VEL, 0.0);
          break;
        }
        case KEY_D:
        {
          ROS_DEBUG("Turn Right");
          moveBase(pub_base_twist, 0.0, -ANGULAR_VEL);
          break;
        }
        case KEY_A:
        {
          ROS_DEBUG("Turn Left");
          moveBase(pub_base_twist, 0.0, +ANGULAR_VEL);
          break;
        }
        case KEY_1:
        {
          ROS_DEBUG("Joint1 Right");
          moveArm(pub_joint_traj, JOINT1_NAME, JOINT_MIN, joint1_pos_);
          break;
        }
        case KEY_2:
        {
          ROS_DEBUG("Joint1 Left");
          moveArm(pub_joint_traj, JOINT1_NAME, JOINT_MAX, joint1_pos_);
          break;
        }
        case KEY_3:
        {
          ROS_DEBUG("Joint2 Up");
          moveArm(pub_joint_traj, JOINT2_NAME, JOINT_MIN, joint2_pos_);
          break;
        }
        case KEY_4:
        {
          ROS_DEBUG("Joint2 Down");
          moveArm(pub_joint_traj, JOINT2_NAME, JOINT_MAX, joint2_pos_);
          break;
        }
        case KEY_5:
        {
          ROS_DEBUG("Joint3 Up");
          moveArm(pub_joint_traj, JOINT3_NAME, JOINT_MIN, joint3_pos_);
          break;
        }
        case KEY_6:
        {
          ROS_DEBUG("Joint3 Down");
          moveArm(pub_joint_traj, JOINT3_NAME, JOINT_MAX, joint3_pos_);
          break;
        }
        case KEY_7:
        {
          ROS_DEBUG("Joint4 Up");
          moveArm(pub_joint_traj, JOINT4_NAME, JOINT_MIN, joint4_pos_);
          break;
        }
        case KEY_8:
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
        case KEY_H:
        {
          ROS_DEBUG("Show Help");
          showHelp();
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
  SIGVerseTb3OpenManipulatorGraspingTeleopKey grasping_teleop_key;

  grasping_teleop_key.keyLoop(argc, argv);

  return(EXIT_SUCCESS);
}

