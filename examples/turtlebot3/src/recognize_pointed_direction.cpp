#include <cstdio>
#include <cmath>
#include <csignal>
#include <limits>
#include <unistd.h>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class SIGVerseTb3RecognizePointedDirection
{
private:
  const int RIGHT_SHOULDER = 7;
//  const int RIGHT_ELBOW    = 9;
  const int RIGHT_WRIST    = 11;
  const std::string JOINT1_NAME = "joint1";
  const std::string INSTRUCTION_MESSAGE = "Rotate the arm to this side";

public:
  SIGVerseTb3RecognizePointedDirection();

  void run(int argc, char** argv);

private:
  static void ros_sigint_handler([[maybe_unused]] int sig);

  void yolo_detection_callback(const yolo_msgs::msg::DetectionArray::SharedPtr detection_array);
  bool compute_pointing_floor_intersection(const geometry_msgs::msg::Point& shoulder, const geometry_msgs::msg::Point& wrist, double floor_z, geometry_msgs::msg::Point& floor_hit_point);
  void publish_debug_markers(const std::string& frame_id, const geometry_msgs::msg::Point& hit_point);

  void instruction_callback(const std_msgs::msg::String::SharedPtr instruction_message);
  void rotate_arm();
  void move_arm(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, const std::string &name, const double position);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_trajectory_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;

  std::map<int, geometry_msgs::msg::Point> keypoint_map_;
  geometry_msgs::msg::Point hit_point_;
  double floor_z_;
};

SIGVerseTb3RecognizePointedDirection::SIGVerseTb3RecognizePointedDirection()
{
  hit_point_.x = std::numeric_limits<double>::quiet_NaN();
}

void SIGVerseTb3RecognizePointedDirection::ros_sigint_handler([[maybe_unused]] int sig)
{
  rclcpp::shutdown();
}

void SIGVerseTb3RecognizePointedDirection::yolo_detection_callback(const yolo_msgs::msg::DetectionArray::SharedPtr detection_array)
{
  if (detection_array->detections.empty()) { return; }

  // Find the detection with the highest score
  const yolo_msgs::msg::Detection* best_detection = nullptr;
  double best_score = -std::numeric_limits<double>::infinity();

  for (const auto &detection : detection_array->detections)
  {
    if (detection.score > best_score) 
    {
      best_score = detection.score;
      best_detection = &detection;
    }
  }

  for (const auto &keypoint : best_detection->keypoints3d.data) 
  {
    keypoint_map_[keypoint.id] = keypoint.point;
  }

  // Update hit_point_
  if (compute_pointing_floor_intersection(keypoint_map_[RIGHT_SHOULDER], keypoint_map_[RIGHT_WRIST], floor_z_, hit_point_)) 
  {
//    RCLCPP_INFO(node_->get_logger(), "Hit Position=(%.3f, %.3f, %.3f)", hit_point_.x, hit_point_.y, hit_point_.z);
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Couldn't compute pointing floor intersection");
  }

  publish_debug_markers(best_detection->keypoints3d.frame_id, hit_point_);
}

/**
 * @brief Compute intersection of the ray (shoulder -> wrist) with floor z=floor_z.
 * @param shoulder  Shoulder position in base_link (m)
 * @param wrist     Wrist position in base_link (m)
 * @param floor_z   Floor height (e.g., -0.5)
 * @param floor_hit_point  Output: intersection point with the floor
 * @return true if intersection exists; false otherwise
 */
bool SIGVerseTb3RecognizePointedDirection::compute_pointing_floor_intersection(
  const geometry_msgs::msg::Point& shoulder, const geometry_msgs::msg::Point& wrist, double floor_z, geometry_msgs::msg::Point& floor_hit_point)
{
  if (wrist.z >= shoulder.z){ return false; }
  if (floor_z >= wrist.z)   { return false; }

  const double dx = wrist.x - shoulder.x;
  const double dy = wrist.y - shoulder.y;
  const double dz = wrist.z - shoulder.z;

  if (std::fabs(dz) < 1e-9) { return false; }

  // Parameter to reach z = floor_z along the ray from the shoulder toward the wrist
  const double t = (floor_z - shoulder.z) / dz;

  floor_hit_point.x = shoulder.x + t * dx;
  floor_hit_point.y = shoulder.y + t * dy;
  floor_hit_point.z = floor_z;

  return true;
}

void SIGVerseTb3RecognizePointedDirection::publish_debug_markers(const std::string& frame_id, const geometry_msgs::msg::Point& hit_point)
{
  visualization_msgs::msg::MarkerArray markerArray;
  const auto stamp = rclcpp::Time(0);

  // Keypoints: green spheres
  visualization_msgs::msg::Marker marker_keypoints;
  marker_keypoints.header.frame_id = frame_id;
  marker_keypoints.header.stamp    = stamp;
  marker_keypoints.ns   = "keypoints";
  marker_keypoints.id   = 0;  // Keep constant to overwrite
  marker_keypoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker_keypoints.action = visualization_msgs::msg::Marker::ADD;
  marker_keypoints.scale = []{ geometry_msgs::msg::Vector3 v; v.x=0.05; v.y=0.05; v.z=0.05; return v; }();
  marker_keypoints.color = []{ std_msgs::msg::ColorRGBA c; c.r=0.0f; c.g=1.0f; c.b=0.0f; c.a=1.0f; return c; }(); // green
  marker_keypoints.pose.orientation.w = 1.0;
  marker_keypoints.points.reserve(keypoint_map_.size());
  for (const auto& kv : keypoint_map_) marker_keypoints.points.push_back(kv.second);
  markerArray.markers.push_back(marker_keypoints);

  // Hit point: red sphere
  visualization_msgs::msg::Marker marker_hit_point;
  marker_hit_point.header.frame_id = frame_id;
  marker_hit_point.header.stamp    = stamp;
  marker_hit_point.ns   = "hit_point";
  marker_hit_point.id   = 0;  // Keep constant to overwrite
  marker_hit_point.type = visualization_msgs::msg::Marker::SPHERE;
  marker_hit_point.action = visualization_msgs::msg::Marker::ADD;
  marker_hit_point.scale = []{ geometry_msgs::msg::Vector3 v; v.x=0.1; v.y=0.1; v.z=0.1; return v; }();
  marker_hit_point.color = []{ std_msgs::msg::ColorRGBA c; c.r=1.0f; c.g=0.0f; c.b=0.0f; c.a=1.0f; return c; }(); // red
  marker_hit_point.pose.orientation.w = 1.0;
  marker_hit_point.pose.position = hit_point;
  markerArray.markers.push_back(marker_hit_point);

  pub_debug_markers_->publish(markerArray);
}


void SIGVerseTb3RecognizePointedDirection::instruction_callback(const std_msgs::msg::String::SharedPtr instruction_message)
{
  if(instruction_message->data==INSTRUCTION_MESSAGE)
  {
    RCLCPP_INFO(node_->get_logger(), "Received Instruction Message");
//    RCLCPP_INFO(node_->get_logger(), "Hit Position=(%.3f, %.3f, %.3f)", hit_point_.x, hit_point_.y, hit_point_.z);
    rotate_arm();
  }
}

void SIGVerseTb3RecognizePointedDirection::rotate_arm()
{
  if (std::isnan(hit_point_.x)) 
  {
    RCLCPP_WARN(node_->get_logger(), "the hit_point_ is uninitialized.");
    return;
  }

  // Rotate the arm
  if     (hit_point_.y > +0.4){ move_arm(pub_joint_trajectory_, JOINT1_NAME, +0.50); }
  else if(hit_point_.y >  0.0){ move_arm(pub_joint_trajectory_, JOINT1_NAME, +0.25); }
  else if(hit_point_.y > -0.4){ move_arm(pub_joint_trajectory_, JOINT1_NAME, -0.25); }
  else                        { move_arm(pub_joint_trajectory_, JOINT1_NAME, -0.50); }
}

void SIGVerseTb3RecognizePointedDirection::move_arm(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, const std::string &name, const double position)
{
  std::vector<std::string> names;
  names.push_back(name);

  std::vector<double> positions;
  positions.push_back(position);

  builtin_interfaces::msg::Duration duration;
  duration.sec = 2;
  duration.nanosec = 0;

  trajectory_msgs::msg::JointTrajectory joint_trajectory;

  trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

  joint_trajectory.points.push_back(arm_joint_point);

  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = duration;

  publisher->publish(joint_trajectory);
}


void SIGVerseTb3RecognizePointedDirection::run(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  node_ = rclcpp::Node::make_shared("tb3_omc_recognize_pointed_direction");

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, ros_sigint_handler);

  floor_z_ = node_->declare_parameter<double>("floor_z", -0.5);

  rclcpp::Rate loop_rate(10);

  pub_joint_trajectory_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/tb3/joint_trajectory", 10);
  pub_debug_markers_    = node_->create_publisher<visualization_msgs::msg::MarkerArray> ("/tb3/debug_markers", 10);

  auto sub_instruction     = node_->create_subscription<std_msgs::msg::String>             ("/tb3/instruction", 10, std::bind(&SIGVerseTb3RecognizePointedDirection::instruction_callback, this, std::placeholders::_1));
  auto sub_yolo_detections = node_->create_subscription<yolo_msgs::msg::DetectionArray>    ("/yolo_human_pose/detections_3d", 10, std::bind(&SIGVerseTb3RecognizePointedDirection::yolo_detection_callback, this, std::placeholders::_1));

  sleep(1);

  rclcpp::spin(node_);

  return;
}


int main(int argc, char** argv)
{
  SIGVerseTb3RecognizePointedDirection recognize_pointed_direction;

  recognize_pointed_direction.run(argc, argv);

  return(EXIT_SUCCESS);
}
