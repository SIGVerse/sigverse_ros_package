#include <cstdio>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <trajectory_msgs/JointTrajectory.h>

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

  static void rosSigintHandler(int sig);

  void instructionCallback(const std_msgs::String::ConstPtr& instruction_message);
  void depthImageCallback (const sensor_msgs::Image::ConstPtr& image);

  void rotateArm();
  float calcSlope(const std::vector<float> &x, const std::vector<float> &y);
  void moveArm(ros::Publisher &publisher, const std::string &name, const double position);

  ros::Publisher pub_joint_traj_;

  uint32_t depth_width_, depth_height_;
  uint8_t is_bigendian_;
  std::vector<uint8_t> depth_data_;
};


SIGVerseTb3RecognizePointedDirection::SIGVerseTb3RecognizePointedDirection()
{
  depth_width_  = 0;
  depth_height_ = 0;
  is_bigendian_ = 0;
}


void SIGVerseTb3RecognizePointedDirection::rosSigintHandler(int sig)
{
  ros::shutdown();
}


void SIGVerseTb3RecognizePointedDirection::instructionCallback(const std_msgs::String::ConstPtr& instruction_message)
{
  if(instruction_message->data==INSTRUCTION_MESSAGE)
  {
    rotateArm();
  }
}


void SIGVerseTb3RecognizePointedDirection::depthImageCallback(const sensor_msgs::Image::ConstPtr& image)
{
  depth_width_  = image->width;
  depth_height_ = image->height;
  is_bigendian_ = image->is_bigendian;

//  puts(("is_bigendian_=" + std::to_string(image->is_bigendian)).c_str());

  uint32_t data_size = image->step * image->height;

  if(depth_data_.size()!=data_size){ depth_data_.resize(data_size); }

  memcpy(&depth_data_[0], &image->data[0], data_size * sizeof(uint8_t));
}


void SIGVerseTb3RecognizePointedDirection::rotateArm()
{
  if(depth_width_==0){ return; }

  // Get depth data
  uint16_t depth_data[depth_height_][depth_width_];

  for(int j=0; j<depth_height_; j++)
  {
    for(int i=0; i<depth_width_; i++)
    {
      int index = 2 * (depth_width_ * j + i);

      if(is_bigendian_==0)
      {
        depth_data[j][i] = ((uint16_t)depth_data_[index]) | ((uint16_t)depth_data_[index+1] << 8);
      }
      else
      {
        depth_data[j][i] = ((uint16_t)depth_data_[index] << 8) | ((uint16_t)depth_data_[index+1]);
      }
    }
  }

  // Intermediate calculation for the calculation of the arm slope of avatar
  float depth_total[depth_width_];
  float row_position[depth_width_];

  int height_max = depth_height_ * 2 / 3; // Exclude the lower third
  float depth_max = 750; // 750[mm]

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

//  puts(("slope=" + std::to_string(slope)).c_str());

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


void SIGVerseTb3RecognizePointedDirection::moveArm(ros::Publisher &publisher, const std::string &name, const double position)
{
  std::vector<std::string> names;
  names.push_back(name);

  std::vector<double> positions;
  positions.push_back(position);

  ros::Duration duration;
  duration.sec = 2;

  trajectory_msgs::JointTrajectory joint_trajectory;

  trajectory_msgs::JointTrajectoryPoint arm_joint_point;

  joint_trajectory.points.push_back(arm_joint_point);

  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = duration;

  publisher.publish(joint_trajectory);
}


void SIGVerseTb3RecognizePointedDirection::run(int argc, char** argv)
{
  ros::init(argc, argv, "tb3_omc_recognize_pointed_direction", ros::init_options::NoSigintHandler);

  ros::NodeHandle node_handle;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(10);

  std::string sub_instruction_topic_name;
  std::string sub_depth_image_topic_name;
  std::string pub_joint_trajectory_topic_name;

  node_handle.param<std::string>("recognize_pointed_direction/sub_instruction_topic_name",      sub_instruction_topic_name,      "/tb3omc/instruction");
  node_handle.param<std::string>("recognize_pointed_direction/sub_depth_image_topic_name",      sub_depth_image_topic_name,      "/camera/depth/image_raw");
  node_handle.param<std::string>("recognize_pointed_direction/pub_joint_trajectory_topic_name", pub_joint_trajectory_topic_name, "/tb3omc/joint_trajectory");

  ros::Subscriber sub_instruction = node_handle.subscribe<std_msgs::String>(sub_instruction_topic_name, 10, &SIGVerseTb3RecognizePointedDirection::instructionCallback, this);
  ros::Subscriber sub_depth_image = node_handle.subscribe                  (sub_depth_image_topic_name, 10, &SIGVerseTb3RecognizePointedDirection::depthImageCallback, this);

  pub_joint_traj_ = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_joint_trajectory_topic_name, 10);

  sleep(1);

  ros::spin();

  return;
}


int main(int argc, char** argv)
{
  SIGVerseTb3RecognizePointedDirection recognize_pointed_direction;

  recognize_pointed_direction.run(argc, argv);

  return(EXIT_SUCCESS);
}

