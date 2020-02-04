#include <cstdio>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <mutex>
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
  SIGVerseTb3OpenManipulatorGraspingTeleopKey();

  void keyLoop(int argc, char** argv);

private:

  static void rosSigintHandler(int sig);
  static int  canReceiveKey( const int fd );

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state);
  void moveBase(ros::Publisher &publisher, const double linear_x, const double angular_z);
  void moveArm(ros::Publisher &publisher, const std::string &name, const double position, const double current_pos);
  void moveHand(ros::Publisher &publisher, const double position, const double current_pos);
  void stopJoints(ros::Publisher &publisher, const int duration_sec);

  static int calcTrajectoryDuration(const double val, const double current_val);

  void showHelp();

  // Current positions that is updated by JointState
  double joint1_pos1_, joint2_pos1_, joint3_pos1_, joint4_pos1_, grip_joint_pos1_;
  double joint1_pos2_, joint2_pos2_, joint3_pos2_, joint4_pos2_;
//  clock_t joint_state_time1_, joint_state_time2_;
};


SIGVerseTb3OpenManipulatorGraspingTeleopKey::SIGVerseTb3OpenManipulatorGraspingTeleopKey()
{
  joint1_pos1_ = 0.0; joint2_pos1_ = 0.0; joint3_pos1_ = 0.0; joint4_pos1_ = 0.0; grip_joint_pos1_ = 0.0;
  joint1_pos2_ = 0.0; joint2_pos2_ = 0.0; joint3_pos2_ = 0.0; joint4_pos2_ = 0.0;
//  joint_state_time2_ = clock();
//  joint_state_time1_ = clock();
}


void SIGVerseTb3OpenManipulatorGraspingTeleopKey::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int SIGVerseTb3OpenManipulatorGraspingTeleopKey::canReceiveKey( const int fd )
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

//  joint_state_time2_ = joint_state_time1_;
//  joint_state_time1_ = clock();

  for(int i=0; i<joint_state->name.size(); i++)
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

//  double coef = (double)(clock()-joint_state_time2_)/(joint_state_time1_-joint_state_time2_);

  std::vector<double> positions;
//  positions.push_back(coef*(joint1_pos1_-joint1_pos2_)+joint1_pos2_);
//  positions.push_back(coef*(joint2_pos1_-joint2_pos2_)+joint2_pos2_);
//  positions.push_back(coef*(joint3_pos1_-joint3_pos2_)+joint3_pos2_);
//  positions.push_back(coef*(joint4_pos1_-joint4_pos2_)+joint4_pos2_);
  positions.push_back(2.0*joint1_pos1_-joint1_pos2_);
  positions.push_back(2.0*joint2_pos1_-joint2_pos2_);
  positions.push_back(2.0*joint3_pos1_-joint3_pos2_);
  positions.push_back(2.0*joint4_pos1_-joint4_pos2_);
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

  ros::init(argc, argv, "tb3_omc_teleop_key", ros::init_options::NoSigintHandler);

  ros::NodeHandle node_handle;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(10);

  std::string sub_joint_state_topic_name;
  std::string pub_base_twist_topic_name;
  std::string pub_joint_trajectory_topic_name;

  node_handle.param<std::string>("grasping_teleop_key/sub_joint_state_topic_name",      sub_joint_state_topic_name,      "/tb3omc/joint_state");
  node_handle.param<std::string>("grasping_teleop_key/pub_twist_topic_name",            pub_base_twist_topic_name,       "/tb3omc/cmd_vel");
  node_handle.param<std::string>("grasping_teleop_key/pub_joint_trajectory_topic_name", pub_joint_trajectory_topic_name, "/tb3omc/joint_trajectory");

  ros::Subscriber sub_joint_state = node_handle.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &SIGVerseTb3OpenManipulatorGraspingTeleopKey::jointStateCallback, this);
  ros::Publisher pub_base_twist = node_handle.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
  ros::Publisher pub_joint_traj = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_joint_trajectory_topic_name, 10);

  sleep(2);

  showHelp();

  while (ros::ok())
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
          ROS_DEBUG("Stop");
          moveBase(pub_base_twist, 0.0, 0.0);
          stopJoints(pub_joint_traj, 1.0);
          break;
        }
        case KEY_W:
        case KEYCODE_UP:
        {
          ROS_DEBUG("Go Forward");
          moveBase(pub_base_twist, +LINEAR_VEL, 0.0);
          break;
        }
        case KEY_S:
        case KEYCODE_DOWN:
        {
          ROS_DEBUG("Go Back");
          moveBase(pub_base_twist, -LINEAR_VEL, 0.0);
          break;
        }
        case KEY_D:
        case KEYCODE_RIGHT:
        {
          ROS_DEBUG("Turn Right");
          moveBase(pub_base_twist, 0.0, -ANGULAR_VEL);
          break;
        }
        case KEY_A:
        case KEYCODE_LEFT:
        {
          ROS_DEBUG("Turn Left");
          moveBase(pub_base_twist, 0.0, +ANGULAR_VEL);
          break;
        }
        case KEY_U:
        {
          ROS_DEBUG("Rotate Arm - Upward");
          moveArm(pub_joint_traj, JOINT2_NAME, 0.0, joint2_pos1_);
          moveArm(pub_joint_traj, JOINT3_NAME, 0.0, joint3_pos1_);
          moveArm(pub_joint_traj, JOINT4_NAME, 0.0, joint4_pos1_);
          break;
        }
        case KEY_J:
        {
          ROS_DEBUG("Rotate Arm - Horizontal");
          moveArm(pub_joint_traj, JOINT2_NAME, +1.57, joint2_pos1_);
          moveArm(pub_joint_traj, JOINT3_NAME, -1.57, joint3_pos1_);
          moveArm(pub_joint_traj, JOINT4_NAME, +0.26, joint4_pos1_);
          break;
        }
        case KEY_M:
        {
          ROS_DEBUG("Rotate Arm - Downward");
          moveArm(pub_joint_traj, JOINT2_NAME, +1.75, joint2_pos1_);
          moveArm(pub_joint_traj, JOINT3_NAME, -1.57, joint3_pos1_);
          moveArm(pub_joint_traj, JOINT4_NAME, +0.26, joint4_pos1_);
          break;
        }
        case KEY_1:
        {
          ROS_DEBUG("Joint1 Right");
          moveArm(pub_joint_traj, JOINT1_NAME, JOINT_MIN, joint1_pos1_);
          break;
        }
        case KEY_2:
        {
          ROS_DEBUG("Joint1 Left");
          moveArm(pub_joint_traj, JOINT1_NAME, JOINT_MAX, joint1_pos1_);
          break;
        }
        case KEY_3:
        {
          ROS_DEBUG("Joint2 Up");
          moveArm(pub_joint_traj, JOINT2_NAME, JOINT_MIN, joint2_pos1_);
          break;
        }
        case KEY_4:
        {
          ROS_DEBUG("Joint2 Down");
          moveArm(pub_joint_traj, JOINT2_NAME, JOINT_MAX, joint2_pos1_);
          break;
        }
        case KEY_5:
        {
          ROS_DEBUG("Joint3 Up");
          moveArm(pub_joint_traj, JOINT3_NAME, JOINT_MIN, joint3_pos1_);
          break;
        }
        case KEY_6:
        {
          ROS_DEBUG("Joint3 Down");
          moveArm(pub_joint_traj, JOINT3_NAME, JOINT_MAX, joint3_pos1_);
          break;
        }
        case KEY_7:
        {
          ROS_DEBUG("Joint4 Up");
          moveArm(pub_joint_traj, JOINT4_NAME, JOINT_MIN, joint4_pos1_);
          break;
        }
        case KEY_8:
        {
          ROS_DEBUG("Joint4 Down");
          moveArm(pub_joint_traj, JOINT4_NAME, JOINT_MAX, joint4_pos1_);
          break;
        }
        case KEY_O:
        {
          ROS_DEBUG("Hand Open");
          moveHand(pub_joint_traj, GRIP_MIN, grip_joint_pos1_);
          break;
        }
        case KEY_C:
        {
          ROS_DEBUG("Hand Close");
          moveHand(pub_joint_traj, GRIP_MAX, grip_joint_pos1_);
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

