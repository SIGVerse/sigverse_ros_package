#ifndef SIGVERSE_TURTLEBOT3_OPEN_MANIPULATOR_GRASPING_AUTO_HPP
#define SIGVERSE_TURTLEBOT3_OPEN_MANIPULATOR_GRASPING_AUTO_HPP

#include <cstdio>
#include <cmath>
#include <cstring>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <chrono>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

using namespace std::chrono;

class SIGVerseTb3OpenManipulatorGraspingAuto
{
public:
  enum GraspingStage
  {
    MoveArm = 0,
    WaitMoveArm,
    Grasp,
    WaitGrasp,
    UpArm,
    WaitUpArm,
  };

private:
  static const char KEY_1 = 0x31;
  static const char KEY_2 = 0x32;
  static const char KEY_3 = 0x33;
  static const char KEY_4 = 0x34;
  static const char KEY_5 = 0x35;

  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;

  static const char KEY_A = 0x61;
  static const char KEY_D = 0x64;
  static const char KEY_H = 0x68;
  static const char KEY_L = 0x6c;
  static const char KEY_O = 0x6f;
  static const char KEY_S = 0x73;
  static const char KEY_W = 0x77;

  static const char KEYCODE_SPACE  = 0x20;


  const std::string LINK1_NAME = "link1";
  const std::string LINK3_NAME = "link3";
  const std::string LINK4_NAME = "link4";
  const std::string LINK5_NAME = "link5";
  const std::string CAMERA_DEPTH_OPTICAL_FRAME_NAME = "camera_depth_optical_frame";

  const std::string TARGET_NAME = "target";

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

  const std::string GRASPING_TARGET1_NAME = "clock";
  const std::string GRASPING_TARGET2_NAME = "cup";
  const std::string GRASPING_TARGET3_NAME = "apple";
  const std::string GRASPING_TARGET4_NAME = "teddy bear";
  const std::string GRASPING_TARGET5_NAME = "person";

  const double PROBABILITY_THRESHOLD = 0.3;

  const int OBJECTS_INFO_UPDATING_INTERVAL = 500; //[ms]

  const int MAX_OBJECTS_NUM = 10;

public:
  SIGVerseTb3OpenManipulatorGraspingAuto();

  void keyLoop(int argc, char** argv);

private:

  static void rosSigintHandler(int sig);
  static int  canReceiveKey( const int fd );

  void jointStateCallback   (const sensor_msgs::JointState::ConstPtr& joint_state);
  void rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info);
  void boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bounding_boxes);
  void pointCloudCallback   (const sensor_msgs::PointCloud2::ConstPtr& point_cloud);

  void moveBase(ros::Publisher &publisher, const double linear_x, const double angular_z);
  void moveArm(ros::Publisher &publisher, const std::string &name, const double position, const double current_pos);
  void moveHand(ros::Publisher &publisher, const double position, const double current_pos);
  void stopJoints(ros::Publisher &publisher, const int duration_sec);
  bool findGraspingTarget(geometry_msgs::Vector3 &point_cloud_pos, const std::string &target_name);
  bool moveArmTowardObject(tf::TransformBroadcaster &tf_broadcaster, tf::TransformListener &tf_listener, ros::Publisher &pub_joint_traj, const std::string &target_name);

  void initializePosture(ros::Publisher &pub_joint_traj);

  template <class T> static T clamp(const T val, const T min, const T max);
  static int calcTrajectoryDuration(const double val, const double current_val);
  static bool get3dPositionFromScreenPosition(geometry_msgs::Vector3 &position3d, const sensor_msgs::PointCloud2 point_cloud, const int x, const int y);
  static bool isWaiting(time_point<system_clock> &latest_stage_time, const int wait_duration_milli);
  static void goNext(time_point<system_clock> &latest_stage_time, GraspingStage &stage);

  std::string getDetectedObjectsList();
  void showDetectedObjectsList();
  void showHelp();

  // Current positions that is updated by JointState
  double joint1_pos_, joint2_pos_, joint3_pos_, joint4_pos_, grip_joint_pos_;

  int rgb_camera_height_, rgb_camera_width_;

  time_point<system_clock> latest_time_of_bounding_boxes_;
  time_point<system_clock> latest_time_of_point_cloud_;

  darknet_ros_msgs::BoundingBoxes bounding_boxes_data_;
  sensor_msgs::PointCloud2        point_cloud_data_;
};


SIGVerseTb3OpenManipulatorGraspingAuto::SIGVerseTb3OpenManipulatorGraspingAuto()
{
  joint1_pos_ = 0.0;
  joint2_pos_ = 0.0;
  joint3_pos_ = 0.0;
  joint4_pos_ = 0.0;
  grip_joint_pos_ = 0.0;

  rgb_camera_height_ = 0;
  rgb_camera_width_  = 0;

  latest_time_of_bounding_boxes_ = system_clock::now();
  latest_time_of_point_cloud_    = system_clock::now();

  bounding_boxes_data_.bounding_boxes.reserve(MAX_OBJECTS_NUM);
}


void SIGVerseTb3OpenManipulatorGraspingAuto::rosSigintHandler(int sig)
{
  ros::shutdown();
}

int SIGVerseTb3OpenManipulatorGraspingAuto::canReceiveKey( const int fd )
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


template <class T>
T SIGVerseTb3OpenManipulatorGraspingAuto::clamp(const T val, const T min, const T max)
{
  return std::min<T>(std::max<T>(min, val), max);
}


int SIGVerseTb3OpenManipulatorGraspingAuto::calcTrajectoryDuration(const double val, const double current_val)
{
  return std::max<int>((int)(std::abs(val - current_val) / 0.5), 1);
}


/**
 * Get the 3D position of PointCloud using the 2D position of the screen.
 * x: 0 to 479,  y: 0 to 359
 */
bool SIGVerseTb3OpenManipulatorGraspingAuto::get3dPositionFromScreenPosition(geometry_msgs::Vector3 &position3d, const sensor_msgs::PointCloud2 point_cloud, const int x, const int y)
{
  if(point_cloud.header.seq==0)
  {
    puts("No point cloud data.");
    return false;
  }

  // -- PointCloud2 memo --
  // height: 360, width: 480
  // point_step=16, row_step=7680(=16*480)
  int point_data_start_position = y * point_cloud.row_step + x * point_cloud.point_step;

  int xpos = point_data_start_position + point_cloud.fields[0].offset;
  int ypos = point_data_start_position + point_cloud.fields[1].offset;
  int zpos = point_data_start_position + point_cloud.fields[2].offset;

  float pos3d_x, pos3d_y, pos3d_z;

  memcpy(&pos3d_x, &point_cloud.data[xpos], sizeof(float));
  memcpy(&pos3d_y, &point_cloud.data[ypos], sizeof(float));
  memcpy(&pos3d_z, &point_cloud.data[zpos], sizeof(float));

  position3d.x = (double)pos3d_x;
  position3d.y = (double)pos3d_y;
  position3d.z = (double)pos3d_z;

  if(std::isnan(position3d.x) || std::isnan(position3d.y) || std::isnan(position3d.z))
  {
//    puts("Point cloud data is nan.");
    return false;
  }

  return true;
}


bool SIGVerseTb3OpenManipulatorGraspingAuto::isWaiting(time_point<system_clock> &latest_stage_time, const int wait_duration_milli)
{
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(system_clock::now() - latest_stage_time).count();

  if(duration < wait_duration_milli){ return true; }

  return false;
}


void SIGVerseTb3OpenManipulatorGraspingAuto::goNext(time_point<system_clock> &latest_stage_time, GraspingStage &stage)
{
  latest_stage_time = system_clock::now();

  stage = static_cast<GraspingStage>(static_cast<int>(stage)+1);
}


#endif // SIGVERSE_TURTLEBOT3_OPEN_MANIPULATOR_GRASPING_AUTO_HPP
