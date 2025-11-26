#ifndef SIGVERSE_TB3_GRASPING_AUTO_HPP
#define SIGVERSE_TB3_GRASPING_AUTO_HPP

#include <cstdio>
#include <cmath>
#include <optional>
#include <mutex>
#include <atomic>
#include <locale.h>
#include <limits>
#include <unistd.h>
#include <termios.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ncurses.h> // Include ncurses after MoveIt headers to avoid macro conflicts

using namespace std::chrono;

class SIGVerseTb3GraspingAuto
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
  const std::string LINK1_NAME = "link1";
  const std::string LINK3_NAME = "link3";
  const std::string LINK4_NAME = "link4";
  const std::string LINK5_NAME = "link5";
  const std::string CAMERA_DEPTH_OPTICAL_FRAME_NAME = "zedm_left_camera_optical_frame";

  const std::string TARGET_NAME = "target";

  const std::string JOINT1_NAME = "joint1";
  const std::string JOINT2_NAME = "joint2";
  const std::string JOINT3_NAME = "joint3";
  const std::string JOINT4_NAME = "joint4";

  const std::string GRIP_JOINT_NAME     = "gripper_left_joint";
  const std::string GRIP_JOINT_SUB_NAME = "gripper_right_joint";

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
  const int WINDOW_HEADER_HEIGHT = 10;

public:
  SIGVerseTb3GraspingAuto();

  void key_loop(int argc, char** argv);

private:
  void yolo_detection_callback(const yolo_msgs::msg::DetectionArray::SharedPtr detection_array);
  void joint_state_callback   (const sensor_msgs::msg::JointState::SharedPtr joint_state);

  void move_base(const double linear_x, const double angular_z);
  void move_arm(const std::string &name, const double position, const double current_pos);
  void move_gripper(const double position, const double current_pos);
  void stop_joints(const int duration_sec);
  bool find_grasping_target(geometry_msgs::msg::Point &target_pos, const std::string &target_name);
  bool move_arm_toward_object(const std::string &target_name);

  void initialize_posture();

  template <class T> static T clamp(const T val, const T min, const T max);
  static int calc_trajectory_duration(const double val, const double current_val);
  static bool is_waiting(time_point<system_clock> &latest_stage_time, const int wait_duration_milli);
  static void go_next(time_point<system_clock> &latest_stage_time, GraspingStage &stage);

  std::string get_detected_objects_list();
  void display_message_in_window(const std::string& text);
  void publish_debug_markers(const std::string& frame_id, const geometry_msgs::msg::Point& target_pos);
  void show_help();
  void update_window_layout();
  void init_window();
  void resize_window();
  void shutdown_window();

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_trajectory_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;

  std::optional<yolo_msgs::msg::DetectionArray> yolo_objects_;
  mutable std::mutex yolo_mutex_; // Guard access if multi-threaded executor is used

  // Current positions that is updated by JointState
  double joint1_pos_, joint2_pos_, joint3_pos_, joint4_pos_, grip_joint_pos_;

  // Window settings
  WINDOW* win_header_ = nullptr;
  static std::atomic<bool> need_redraw_window_; // set from existing SIGWINCH handler
  int header_height_ = WINDOW_HEADER_HEIGHT;
};


SIGVerseTb3GraspingAuto::SIGVerseTb3GraspingAuto()
{
  joint1_pos_ = 0.0;
  joint2_pos_ = 0.0;
  joint3_pos_ = 0.0;
  joint4_pos_ = 0.0;
  grip_joint_pos_ = 0.0;
}


template <class T>
T SIGVerseTb3GraspingAuto::clamp(const T val, const T min, const T max)
{
  return std::min<T>(std::max<T>(min, val), max);
}


int SIGVerseTb3GraspingAuto::calc_trajectory_duration(const double val, const double current_val)
{
  return std::max<int>((int)(std::abs(val - current_val) / 0.5), 1);
}


bool SIGVerseTb3GraspingAuto::is_waiting(time_point<system_clock> &latest_stage_time, const int wait_duration_milli)
{
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(system_clock::now() - latest_stage_time).count();

  if(duration < wait_duration_milli){ return true; }

  return false;
}


void SIGVerseTb3GraspingAuto::go_next(time_point<system_clock> &latest_stage_time, GraspingStage &stage)
{
  latest_stage_time = system_clock::now();

  stage = static_cast<GraspingStage>(static_cast<int>(stage)+1);
}


#endif // SIGVERSE_TB3_GRASPING_AUTO_HPP
