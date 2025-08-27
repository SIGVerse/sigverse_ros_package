#include <memory>
#include <cmath>
#include <signal.h>
#include <termios.h>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
//#include <tmc_suction/suction_server.hpp>

class SIGVerseHsrTeleopKey
{
private:
  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;

  const std::string MSG_TELL_ME  = "Please tell me";
  const std::string MSG_POINT_IT = "Please point it";

  const std::string ARM_LIFT_JOINT_NAME   = "arm_lift_joint";
  const std::string ARM_FLEX_JOINT_NAME   = "arm_flex_joint";
  const std::string ARM_ROLL_JOINT_NAME   = "arm_roll_joint";
  const std::string WRIST_FLEX_JOINT_NAME = "wrist_flex_joint";
  const std::string WRIST_ROLL_JOINT_NAME = "wrist_roll_joint";

public:
  SIGVerseHsrTeleopKey();

  static void rosSigintHandler([[maybe_unused]] int sig);
  static int  canReceive(int fd);

  void messageCallback(const std_msgs::msg::String::SharedPtr message);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  void sendMessage(const std::string &message);
  void moveBaseTwist(const double linear_x, const double linear_y, const double angular_z);
  void moveBaseJointTrajectory(const double linear_x, const double linear_y, const double theta, const double duration_sec);
  void operateArm(const double arm_lift_pos, const double arm_flex_pos, const double arm_roll_pos, const double wrist_flex_pos, const double wrist_roll_pos, const double duration_sec);
  void operateArm(const std::string &name, const double position, const double duration_sec);
  void operateArmFlex(const double arm_flex_pos, const double wrist_flex_pos);
  double getDurationRot(const double next_pos, const double current_pos);
  void operateHand(const bool is_hand_open);
  void sendSuctionGoal(const bool &sution_on);
//  void suctionResultCallback(const tmc_suction::SuctionControlActionResult::ConstPtr& suction_result);

  void showHelp();
  int run();
  
private:
  // Last position and previous position of arm_lift_joint
  double arm_lift_joint_pos1_;
  double arm_lift_joint_pos2_;
  double arm_flex_joint_pos_;
  double arm_roll_joint_pos_;
  double wrist_flex_joint_pos_;
  double wrist_roll_joint_pos_;

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_base_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_trajectory_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
//  ros::Publisher  pub_suction_goal_;
//  ros::Subscriber sub_suction_result_; 
};


SIGVerseHsrTeleopKey::SIGVerseHsrTeleopKey()
{
  arm_lift_joint_pos1_   = 0.0;
  arm_lift_joint_pos2_   = 0.0;
  arm_flex_joint_pos_    = 0.0;
  arm_roll_joint_pos_    = 0.0;
  wrist_flex_joint_pos_  = 0.0;
  wrist_roll_joint_pos_  = 0.0;
  
  node_ = rclcpp::Node::make_shared("hsr_teleop_key");
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  pub_msg_                = node_->create_publisher<std_msgs::msg::String>                ("/hsrb/message/to_human", 10);
  pub_base_twist_         = node_->create_publisher<geometry_msgs::msg::Twist>            ("/hsrb/command_velocity", 10);
  pub_base_trajectory_    = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/omni_base_controller/command", 10);
  pub_arm_trajectory_     = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
  pub_gripper_trajectory_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/gripper_controller/command", 10);

  sub_msg_         = node_->create_subscription<std_msgs::msg::String>       ("/hsrb/message/to_robot", 10, std::bind(&SIGVerseHsrTeleopKey::messageCallback,    this, std::placeholders::_1));
  sub_joint_state_ = node_->create_subscription<sensor_msgs::msg::JointState>("/hsrb/joint_states",     10, std::bind(&SIGVerseHsrTeleopKey::jointStateCallback, this, std::placeholders::_1));

//  pub_suction_goal_       = node_handle_.advertise<tmc_suction::SuctionControlActionGoal>  ("/hsrb/suction_control/goal", 10, false);
//  sub_suction_result_     = node_handle_.subscribe<tmc_suction::SuctionControlActionResult>("/hsrb/suction_control/result", 10, &SIGVerseHsrTeleopKey::suctionResultCallback, this);
}


void SIGVerseHsrTeleopKey::rosSigintHandler([[maybe_unused]] int sig)
{
  rclcpp::shutdown();
}


int SIGVerseHsrTeleopKey::canReceive( int fd )
{
  fd_set fdset;
  struct timeval timeout;
  FD_ZERO( &fdset );
  FD_SET( fd , &fdset );

  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  return select( fd+1 , &fdset , NULL , NULL , &timeout );
}

void SIGVerseHsrTeleopKey::messageCallback(const std_msgs::msg::String::SharedPtr message)
{
  RCLCPP_INFO(node_->get_logger(), "Subscribe message: %s", message->data.c_str());
}

void SIGVerseHsrTeleopKey::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
  for(size_t i=0; i<joint_state->name.size(); i++)
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
  RCLCPP_INFO(node_->get_logger(), "Send message:%s", message.c_str());

  std_msgs::msg::String string_msg;
  string_msg.data = message;
  pub_msg_->publish(string_msg);
}

void SIGVerseHsrTeleopKey::moveBaseTwist(const double linear_x, const double linear_y, const double angular_z)
{
  geometry_msgs::msg::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_->publish(twist);
}

void SIGVerseHsrTeleopKey::moveBaseJointTrajectory(const double linear_x, const double linear_y, const double theta, const double duration_sec)
{
  if(!tf_buffer_->canTransform("/odom", "/base_footprint", tf2::TimePointZero))
  {
    return;
  }

  geometry_msgs::msg::PointStamped basefootprint_2_target;
  geometry_msgs::msg::PointStamped odom_2_target;
  basefootprint_2_target.header.frame_id = "/base_footprint";
  basefootprint_2_target.header.stamp = node_->get_clock()->now();
  basefootprint_2_target.point.x = linear_x;
  basefootprint_2_target.point.y = linear_y;
  odom_2_target = tf_buffer_->transform(basefootprint_2_target, "odom", tf2::Duration(0)); 

  geometry_msgs::msg::TransformStamped transform;

  try
  {
    transform = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(node_->get_logger(), "Could not transform: %s", ex.what());
    return;
  }

  tf2::Quaternion quat;
  tf2::fromMsg(transform.transform.rotation, quat);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("odom_x");
  joint_trajectory.joint_names.push_back("odom_y");
  joint_trajectory.joint_names.push_back("odom_t");

  trajectory_msgs::msg::JointTrajectoryPoint omni_joint_point;
  omni_joint_point.positions = {odom_2_target.point.x, odom_2_target.point.y, yaw + theta};
  omni_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

  joint_trajectory.points.push_back(omni_joint_point);
  pub_base_trajectory_->publish(joint_trajectory);
}

void SIGVerseHsrTeleopKey::operateArm(const double arm_lift_pos, const double arm_flex_pos, const double arm_roll_pos, const double wrist_flex_pos, const double wrist_roll_pos, const double duration_sec)
{
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back(ARM_LIFT_JOINT_NAME);
  joint_trajectory.joint_names.push_back(ARM_FLEX_JOINT_NAME);
  joint_trajectory.joint_names.push_back(ARM_ROLL_JOINT_NAME);
  joint_trajectory.joint_names.push_back(WRIST_FLEX_JOINT_NAME);
  joint_trajectory.joint_names.push_back(WRIST_ROLL_JOINT_NAME);

  trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

  arm_joint_point.positions = {arm_lift_pos, arm_flex_pos, arm_roll_pos, wrist_flex_pos, wrist_roll_pos};

  arm_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  joint_trajectory.points.push_back(arm_joint_point);
  pub_arm_trajectory_->publish(joint_trajectory);
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
    RCLCPP_DEBUG(node_->get_logger(), "Grasp");
    positions.push_back(-0.105);
  }
  else
  {
    RCLCPP_DEBUG(node_->get_logger(), "Open hand");
    positions.push_back(+1.239);
  }

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = rclcpp::Duration::from_seconds(2);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);
  pub_gripper_trajectory_->publish(joint_trajectory);
}

//void SIGVerseHsrTeleopKey::sendSuctionGoal(const bool &sution_on)
//{
//  tmc_suction::SuctionControlActionGoal goal_msg;
//  goal_msg.goal.timeout = ros::Duration(10.0);
//  goal_msg.goal.suction_on.data = sution_on;
//  pub_suction_goal_.publish(goal_msg);
//}

//void SIGVerseHsrTeleopKey::suctionResultCallback(const tmc_suction::SuctionControlActionResult::ConstPtr& suction_result) 
//{
//  // 3: SUCCEEDED
//  // 4: ABORTED(Timeout)
//  ROS_INFO("Subscribe Suction result: status=%i", suction_result->status.status);
//}
  
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


int SIGVerseHsrTeleopKey::run()
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

  auto logger = node_->get_logger();

  rclcpp::Rate loop_rate(40);

  const float linear_coef  = 0.2f;
  const float angular_coef = 0.5f;

  float move_speed = 1.0f;
  bool is_hand_open = false;
//  bool is_suction_on = false;

  while (rclcpp::ok())
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
        case '1':
        {
          sendMessage(MSG_TELL_ME);
          break;
        }
        case '2':
        {
          sendMessage(MSG_POINT_IT);
          break;
        }
        case KEYCODE_UP:
        {
          RCLCPP_DEBUG(logger, "Go Forward");
          moveBaseTwist(+linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_DOWN:
        {
          RCLCPP_DEBUG(logger, "Go Backward");
          moveBaseTwist(-linear_coef*move_speed, 0.0, 0.0);
          break;
        }
        case KEYCODE_RIGHT:
        {
          RCLCPP_DEBUG(logger, "Go Right");
          moveBaseTwist(0.0, 0.0, -angular_coef*move_speed);
          break;
        }
        case KEYCODE_LEFT:
        {
          RCLCPP_DEBUG(logger, "Go Left");
          moveBaseTwist(0.0, 0.0, +angular_coef*move_speed);
          break;
        }
        case ' ':
        {
          RCLCPP_DEBUG(logger, "Stop");
          moveBaseTwist(0.0, 0.0, 0.0);
          break;
        }
        case 'u':
        {
          RCLCPP_DEBUG(logger, "Move Left Forward");
          moveBaseJointTrajectory(+1.0, +1.0, +M_PI_4, 10);
          break;
        }
        case 'i':
        {
          RCLCPP_DEBUG(logger, "Move Forward");
          moveBaseJointTrajectory(+1.0, 0.0, 0.0, 10);
          break;
        }
        case 'o':
        {
          RCLCPP_DEBUG(logger, "Move Right Forward");
          moveBaseJointTrajectory(+1.0, -1.0, -M_PI_4, 10);
          break;
        }
        case 'j':
        {
          RCLCPP_DEBUG(logger, "Move Left");
          moveBaseJointTrajectory(0.0, +1.0, +M_PI_2, 10);
          break;
        }
        case 'k':
        {
          RCLCPP_DEBUG(logger, "Stop");
          moveBaseJointTrajectory(0.0, 0.0, 0.0, 0.5);
          break;
        }
        case 'l':
        {
          RCLCPP_DEBUG(logger, "Move Right");
          moveBaseJointTrajectory(0.0, -1.0, -M_PI_2, 10);
          break;
        }
        case 'm':
        {
          RCLCPP_DEBUG(logger, "Move Left Backward");
          moveBaseJointTrajectory(-1.0, +1.0, +M_PI_2+M_PI_4, 10);
          break;
        }
        case ',':
        {
          RCLCPP_DEBUG(logger, "Move Backward");
          moveBaseJointTrajectory(-1.0, 0.0, +M_PI, 10);
          break;
        }
        case '.':
        {
          RCLCPP_DEBUG(logger, "Move Right Backward");
          moveBaseJointTrajectory(-1.0, -1.0, -M_PI_2-M_PI_4, 10);
          break;
        }
        case 'q':
        {
          RCLCPP_DEBUG(logger, "Move Speed Up");
          move_speed *= 2;
          if(move_speed > 2  ){ move_speed=2; }
          break;
        }
        case 'z':
        {
          RCLCPP_DEBUG(logger, "Move Speed Down");
          move_speed /= 2;
          if(move_speed < 0.125){ move_speed=0.125; }
          break;
        }
        case 'y':
        {
          RCLCPP_DEBUG(logger, "Up Torso");
          operateArm(ARM_LIFT_JOINT_NAME, 0.69, std::max<int>((int)(std::abs(0.69 - arm_lift_joint_pos1_) / 0.05), 1)/move_speed);
          break;
        }
        case 'h':
        {
          RCLCPP_DEBUG(logger, "Stop Torso");
          operateArm(ARM_LIFT_JOINT_NAME, 2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
          break;
        }
        case 'n':
        {
          RCLCPP_DEBUG(logger, "Down Torso");
          operateArm(ARM_LIFT_JOINT_NAME, 0.0, std::max<int>((int)(std::abs(0.0 - arm_lift_joint_pos1_) / 0.05), 1)/move_speed);
          break;
        }
        //operateArm(const double arm_lift_pos, const double arm_flex_pos, const double wrist_flex_pos, const int duration_sec);
        case 'a':
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Vertical");
          operateArmFlex(0.0, -1.57);
          break;
        }
        case 'b':
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Upward");
          operateArmFlex(-0.785, -0.785);
          break;
        }
        case 'c':
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Horizontal");
          operateArmFlex(-1.57, 0.0);
          break;
        }
        case 'd':
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Downward");
          operateArmFlex(-2.2, 0.35);
          break;
        }
        case 'e':
        {
          RCLCPP_DEBUG(logger, "Rotate Arm - Suction Downward");
          operateArm(arm_lift_joint_pos1_, -2.01, -0.878, -1.108, 0.174, 3);
          break;
        }
        case 'g':
        {
          operateHand(is_hand_open);

          is_hand_open = !is_hand_open;
          break;
        }
//        case KEYCODE_V:
//        {
//          sendSuctionGoal(true);
//          break;
//        }
//        case KEYCODE_W:
//        {
//          sendSuctionGoal(false);
//          break;
//        }  
      }
    }

    rclcpp::spin_some(node_);

    loop_rate.sleep();
  }

  /////////////////////////////////////////////
  // cooked mode
  tcsetattr(kfd, TCSANOW, &cooked);
  /////////////////////////////////////////////

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  SIGVerseHsrTeleopKey hsr_teleop_key;
  return hsr_teleop_key.run();
}

