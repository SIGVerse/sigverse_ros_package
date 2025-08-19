#include <cstdio>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "builtin_interfaces/msg/duration.hpp"

class SIGVerseTb3RecognizePointedDirection
{
private:
  const std::string JOINT1_NAME = "joint1";
  const double JOINT1_ANGLE = 0.5;
  const std::string INSTRUCTION_MESSAGE = "Rotate the arm to this side";

public:
  SIGVerseTb3RecognizePointedDirection();

  void run(int argc, char** argv);

private:

  static void rosSigintHandler([[maybe_unused]] int sig);

  void instructionCallback(const std_msgs::msg::String::SharedPtr instruction_message);
  void depthImageCallback (const sensor_msgs::msg::Image::SharedPtr image);

  void rotateArm();
  float calcSlope(const std::vector<float> &x, const std::vector<float> &y);
  void moveArm(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, const std::string &name, const double position);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_traj_;

  uint32_t depth_width_, depth_height_;
  uint8_t is_bigendian_;
  std::vector<uint8_t> depth_data_;

  rclcpp::Node::SharedPtr node_;
};


SIGVerseTb3RecognizePointedDirection::SIGVerseTb3RecognizePointedDirection()
{
  depth_width_  = 0;
  depth_height_ = 0;
  is_bigendian_ = 0;
}


void SIGVerseTb3RecognizePointedDirection::rosSigintHandler([[maybe_unused]] int sig)
{
  rclcpp::shutdown();
}


void SIGVerseTb3RecognizePointedDirection::instructionCallback(const std_msgs::msg::String::SharedPtr instruction_message)
{
  if(instruction_message->data==INSTRUCTION_MESSAGE)
  {
    rotateArm();
  }
}


void SIGVerseTb3RecognizePointedDirection::depthImageCallback(const sensor_msgs::msg::Image::SharedPtr image)
{
  depth_width_  = image->width;
  depth_height_ = image->height;
  is_bigendian_ = image->is_bigendian;

  uint32_t data_size = image->step * image->height;

  if(depth_data_.size()!=data_size){ depth_data_.resize(data_size); }

  memcpy(&depth_data_[0], &image->data[0], data_size * sizeof(uint8_t));
}


void SIGVerseTb3RecognizePointedDirection::rotateArm()
{
  if(depth_width_==0){ return; }

  // Get depth data
  float depth_data[depth_height_][depth_width_];

  for(int j=0; j<depth_height_; j++)
  {
    for(int i=0; i<depth_width_; i++)
    {
      int idx = 4 * (depth_width_ * j + i);

      if(is_bigendian_==0)
      {
        uint32_t tmp = ((uint32_t)depth_data_[idx]) | ((uint32_t)depth_data_[idx+1]<<8) | ((uint32_t)depth_data_[idx+2]<<16) | ((uint32_t)depth_data_[idx+3]<<24);
        memcpy(&depth_data[j][i], &tmp, 4);
      }
      else
      {
        uint32_t tmp = ((uint32_t)depth_data_[idx]<<24) | ((uint32_t)depth_data_[idx+1]<<16) | ((uint32_t)depth_data_[idx+2]<<8) | ((uint32_t)depth_data_[idx+3]);
        memcpy(&depth_data[j][i], &tmp, 4);
      }
    }
  }
  
  // Intermediate calculation for the calculation of the arm slope of avatar
  float depth_total[depth_width_];
  float row_position[depth_width_];

  int height_max = depth_height_ * 0.4; // Exclude the lower side.
  float depth_max = 0.75; // [m]

  for(int j=0; j<depth_width_; j++)
  {
    // Calc the depth total per column
    depth_total[j] = 0.0f;

    for(int i=0; i<height_max; i++)
    {
      if(depth_data[i][j]!=0 && depth_data[i][j]<depth_max)
      {
        depth_total[j] += 1.0f / depth_data[i][j];
      }
    }

    // Calc the row position of the depth centroid per column
    row_position[j] = 0.0f;

    if(depth_total[j]!=0)
    {
      for(int i=0; i<height_max; i++)
      {
        if(depth_data[i][j]!=0 && depth_data[i][j]<depth_max)
        {
          row_position[j] += i * 1.0f / depth_data[i][j] / depth_total[j];
        }
      }
    }
  }

  // Calc the arm slope of avatar using the least squares method
  std::vector<float> x_list;
  std::vector<float> y_list;

  for(int j=0; j<depth_width_; j++)
  {
    if(row_position[j]!=0)
    {
      x_list.push_back(j);
      y_list.push_back(row_position[j]);
    }
  }

  float slope = calcSlope(x_list, y_list);

  // Rotate the arm
  if(slope < 0.0f)
  {
    moveArm(pub_joint_traj_, JOINT1_NAME, +JOINT1_ANGLE);
  }
  else
  {
    moveArm(pub_joint_traj_, JOINT1_NAME, -JOINT1_ANGLE);
  }
}


float SIGVerseTb3RecognizePointedDirection::calcSlope(const std::vector<float> &x, const std::vector<float> &y)
{
  // Calc a slope using the least squares method

  double a = 0.0;
  double sum_xy = 0.0, sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0;

  int num = x.size();

  for(int i=0; i<num; i++)
  {
    sum_xy += x[i] * y[i];
    sum_x  += x[i];
    sum_y  += y[i];
    sum_x2 += std::pow((double)x[i], 2);
  }

  return (float)((num * sum_xy - sum_x * sum_y) / (num * sum_x2 - std::pow(sum_x, 2)));
}


void SIGVerseTb3RecognizePointedDirection::moveArm(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher, const std::string &name, const double position)
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
  signal(SIGINT, rosSigintHandler);

  rclcpp::Rate loop_rate(10);

  auto sub_instruction = node_->create_subscription<std_msgs::msg::String>("/tb3omc/instruction", 10, std::bind(&SIGVerseTb3RecognizePointedDirection::instructionCallback, this, std::placeholders::_1));
  auto sub_depth_image = node_->create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10, std::bind(&SIGVerseTb3RecognizePointedDirection::depthImageCallback, this, std::placeholders::_1));

  pub_joint_traj_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/tb3omc/joint_trajectory", 10);

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
