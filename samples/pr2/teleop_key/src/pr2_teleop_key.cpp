#include "pr2_teleop_key.hpp"

void SIGVersePr2TeleopKey::rosSigintHandler(int sig)
{
  ros::shutdown();
}


int SIGVersePr2TeleopKey::canReceive( int fd )
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

void SIGVersePr2TeleopKey::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  for(int i=0; i<joint_state->name.size(); i++)
  {
    joint_state_map_[joint_state->name[i]] = joint_state->position[i];
  }
}

void SIGVersePr2TeleopKey::moveBase(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_.publish(twist);
}


void SIGVersePr2TeleopKey::operateHead(const std::string &joint_name, const double add_pos)
{
  // Clamp
  double clamped_next_pos = std::min(std::max(joint_min_map_[joint_name], joint_state_map_[joint_name]+add_pos), joint_max_map_[joint_name]);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("head_pan_joint");
  joint_trajectory.joint_names.push_back("head_tilt_joint");

  trajectory_msgs::JointTrajectoryPoint trajectoryPoint;
  trajectoryPoint.positions = {joint_state_map_["head_pan_joint"], joint_state_map_["head_tilt_joint"]};

  for(int i=0; i<joint_trajectory.joint_names.size(); i++)
  {
    if(joint_trajectory.joint_names[i]==joint_name)
    {
      trajectoryPoint.positions[i] = clamped_next_pos;
    }
  }

  trajectoryPoint.time_from_start = ros::Duration(this->getDurationRot(clamped_next_pos, joint_state_map_[joint_name]));
  joint_trajectory.points.push_back(trajectoryPoint);
  pub_head_trajectory_.publish(joint_trajectory);
}

void SIGVersePr2TeleopKey::operateTorso(const std::string &joint_name, const double add_pos)
{
  // Clamp
  double clamped_next_pos = std::min(std::max(joint_min_map_[joint_name], joint_state_map_[joint_name]+add_pos), joint_max_map_[joint_name]);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back("torso_lift_joint");

  trajectory_msgs::JointTrajectoryPoint trajectoryPoint;
  trajectoryPoint.positions = { clamped_next_pos };

  trajectoryPoint.time_from_start = ros::Duration(this->getDurationPos(clamped_next_pos, joint_state_map_[joint_name]));
  joint_trajectory.points.push_back(trajectoryPoint);
  pub_torso_trajectory_.publish(joint_trajectory);
}

void SIGVersePr2TeleopKey::operateArm(const HandType hand_type, const std::string &joint_type, const double add_pos)
{
  std::string prefix;
  if(hand_type==HandType::Left){ prefix = "l";} else { prefix = "r"; }

  std::string joint_name = prefix + "_" + joint_type;

  // Clamp
  double clamped_next_pos = std::min(std::max(joint_min_map_[joint_name], joint_state_map_[joint_name]+add_pos), joint_max_map_[joint_name]);

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names.push_back(prefix + "_shoulder_pan_joint");
  joint_trajectory.joint_names.push_back(prefix + "_shoulder_lift_joint");
  joint_trajectory.joint_names.push_back(prefix + "_upper_arm_roll_joint");
  joint_trajectory.joint_names.push_back(prefix + "_elbow_flex_joint");
  joint_trajectory.joint_names.push_back(prefix + "_forearm_roll_joint");
  joint_trajectory.joint_names.push_back(prefix + "_wrist_flex_joint");
  joint_trajectory.joint_names.push_back(prefix + "_wrist_roll_joint");

  trajectory_msgs::JointTrajectoryPoint trajectoryPoint;
  trajectoryPoint.positions =
  {
    joint_state_map_[prefix + "_shoulder_pan_joint"],
    joint_state_map_[prefix + "_shoulder_lift_joint"],
    joint_state_map_[prefix + "_upper_arm_roll_joint"],
    joint_state_map_[prefix + "_elbow_flex_joint"],
    joint_state_map_[prefix + "_forearm_roll_joint"],
    joint_state_map_[prefix + "_wrist_flex_joint"],
    joint_state_map_[prefix + "_wrist_roll_joint"]
  };

  for(int i=0; i<joint_trajectory.joint_names.size(); i++)
  {
    if(joint_trajectory.joint_names[i]==joint_name)
    {
      trajectoryPoint.positions[i] = clamped_next_pos;
    }
  }

  trajectoryPoint.time_from_start = ros::Duration(this->getDurationRot(clamped_next_pos, joint_state_map_[joint_name]));
  joint_trajectory.points.push_back(trajectoryPoint);

  if(hand_type==HandType::Left)
  {
    pub_l_arm_trajectory_.publish(joint_trajectory);
  }
  else
  {
    pub_r_arm_trajectory_.publish(joint_trajectory);
  }
}

void SIGVersePr2TeleopKey::operateGripper(const HandType hand_type, const float is_open)
{
  // Clamp
  double next_pos = (is_open)? 0.086 : 0.0;

  pr2_controllers_msgs::Pr2GripperCommand gripperCommand;
  gripperCommand.position = next_pos;
  gripperCommand.max_effort = 1.0;

  if(hand_type==HandType::Left)
  {
    pub_l_gripper_command_.publish(gripperCommand);
  }
  else
  {
    pub_r_gripper_command_.publish(gripperCommand);
  }
}


double SIGVersePr2TeleopKey::getDurationPos(const double next_pos, const double current_pos)
{
  return std::max<double>((std::abs(next_pos - current_pos) * 15), 1.0);
}

double SIGVersePr2TeleopKey::getDurationRot(const double next_pos, const double current_pos)
{
  return std::max<double>((std::abs(next_pos - current_pos) * 3), 1.0);
}

int SIGVersePr2TeleopKey::run()
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

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  ros::Rate loop_rate(50);

  std::string sub_joint_state_topic_name;
  std::string pub_base_twist_topic_name;
  std::string pub_head_trajectory_topic_name;
  std::string pub_torso_trajectory_topic_name;
  std::string pub_l_arm_trajectory_topic_name;
  std::string pub_r_arm_trajectory_topic_name;
  std::string pub_l_gripper_command_topic_name;
  std::string pub_r_gripper_command_topic_name;

  node_handle_.param<std::string>("sub_joint_state_topic_name",      sub_joint_state_topic_name,      "/joint_states");
  node_handle_.param<std::string>("pub_base_twist_topic_name",       pub_base_twist_topic_name,       "/base_controller/command");
  node_handle_.param<std::string>("pub_head_trajectory_topic_name",  pub_head_trajectory_topic_name,  "/head_traj_controller/command");
  node_handle_.param<std::string>("pub_torso_trajectory_topic_name", pub_torso_trajectory_topic_name, "/torso_controller/command");
  node_handle_.param<std::string>("pub_l_arm_trajectory_topic_name", pub_l_arm_trajectory_topic_name, "/l_arm_controller/command");
  node_handle_.param<std::string>("pub_r_arm_trajectory_topic_name", pub_r_arm_trajectory_topic_name, "/r_arm_controller/command");
  node_handle_.param<std::string>("pub_l_gripper_command_topic_name",pub_l_gripper_command_topic_name,"/l_gripper_controller/command");
  node_handle_.param<std::string>("pub_r_gripper_command_topic_name",pub_r_gripper_command_topic_name,"/r_gripper_controller/command");

  sub_joint_state_      = node_handle_.subscribe<sensor_msgs::JointState>(sub_joint_state_topic_name, 10, &SIGVersePr2TeleopKey::jointStateCallback, this);
  pub_base_twist_       = node_handle_.advertise<geometry_msgs::Twist>(pub_base_twist_topic_name, 10);
  pub_head_trajectory_  = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_head_trajectory_topic_name, 10);
  pub_torso_trajectory_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_torso_trajectory_topic_name, 10);
  pub_l_arm_trajectory_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_l_arm_trajectory_topic_name, 10);
  pub_r_arm_trajectory_ = node_handle_.advertise<trajectory_msgs::JointTrajectory>(pub_r_arm_trajectory_topic_name, 10);
  pub_l_gripper_command_= node_handle_.advertise<pr2_controllers_msgs::Pr2GripperCommand>(pub_l_gripper_command_topic_name, 10);
  pub_r_gripper_command_= node_handle_.advertise<pr2_controllers_msgs::Pr2GripperCommand>(pub_r_gripper_command_topic_name, 10);

  const float linear_coef  = 0.2f;
  const float angular_coef = 0.5f;

  float move_speed = 10.0f;

  showHelp();

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

    if(!canReceive(kfd)){ continue; }

    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(EXIT_FAILURE);
    }

    switch(c)
    {
      case KEYCODE_UP:   { moveBase(+linear_coef*move_speed, 0.0, 0.0);  break; }
      case KEYCODE_DOWN: { moveBase(-linear_coef*move_speed, 0.0, 0.0);  break; }
      case KEYCODE_RIGHT:{ moveBase(0.0, 0.0, -angular_coef*move_speed); break; }
      case KEYCODE_LEFT: { moveBase(0.0, 0.0, +angular_coef*move_speed); break; }
      case KEYCODE_SPACE:{ moveBase(0.0, 0.0, 0.0);                      break; }
      case KEYCODE_Q:
      {
        ROS_DEBUG("Move Speed Up");
        move_speed *= 2;
        if(move_speed > 2  ){ move_speed=2; }
        break;
      }
      case KEYCODE_Z:
      {
        ROS_DEBUG("Move Speed Down");
        move_speed /= 2;
        if(move_speed < 0.125){ move_speed=0.125; }
        break;
      }
      case KEYCODE_H:
      {
        showHelpHead();

        bool is_head_stop = false;

        while(!is_head_stop)
        {
          ros::spinOnce();
          loop_rate.sleep();

          if(!canReceive(kfd)){ continue; }

          if(read(kfd, &c, 1) < 0)
          {
            perror("read():");
            exit(EXIT_FAILURE);
          }

          switch(c)
          {
            case KEYCODE_R:{ ROS_DEBUG("Head Pan +");  operateHead("head_pan_joint", +1.0); break; }
            case KEYCODE_F:{ ROS_DEBUG("Head Pan -");  operateHead("head_pan_joint", -1.0); break; }
            case KEYCODE_T:{ ROS_DEBUG("Head Tilt -"); operateHead("head_tilt_joint",-1.0); break; }
            case KEYCODE_G:{ ROS_DEBUG("Head Tilt +"); operateHead("head_tilt_joint",+1.0); break; }

            case KEYCODE_SPACE:{ ROS_DEBUG("Head Stop"); operateHead("head_pan_joint", 0.0); break; }
            case KEYCODE_Q:
            {
              is_head_stop = true;
              showHelp();
              break;
            }
          }
        }
        break;
      }
      case KEYCODE_T:
      {
        showHelpTorso();

        bool is_torso_stop = false;

        while(!is_torso_stop)
        {
          ros::spinOnce();
          loop_rate.sleep();

          if(!canReceive(kfd)){ continue; }

          if(read(kfd, &c, 1) < 0)
          {
            perror("read():");
            exit(EXIT_FAILURE);
          }

          switch(c)
          {
            case KEYCODE_R:{ ROS_DEBUG("Torso lift +"); operateTorso("torso_lift_joint",+0.15); break; }
            case KEYCODE_F:{ ROS_DEBUG("Torso lift -"); operateTorso("torso_lift_joint",-0.15); break; }

            case KEYCODE_SPACE:{ ROS_DEBUG("Torso Stop"); operateTorso("torso_lift_joint",0.0); break; }
            case KEYCODE_Q:
            {
              is_torso_stop = true;
              showHelp();
              break;
            }
          }
        }
        break;
      }
      case KEYCODE_L:
      case KEYCODE_R:
      {
        HandType hand_type;

        if(c==KEYCODE_L){ hand_type = HandType::Left;  showHelpArm("Left Arm"); }
        else            { hand_type = HandType::Right; showHelpArm("Right Arm");}

        bool is_arm_stop = false;

        while(!is_arm_stop)
        {
          ros::spinOnce();
          loop_rate.sleep();

          if(!canReceive(kfd)){ continue; }

          if(read(kfd, &c, 1) < 0)
          {
            perror("read():");
            exit(EXIT_FAILURE);
          }

          switch(c)
          {
            case KEYCODE_W:{ ROS_DEBUG("Shoulder pan +");   operateArm(hand_type, "shoulder_pan_joint",   +1.0); break; }
            case KEYCODE_S:{ ROS_DEBUG("Shoulder pan -");   operateArm(hand_type, "shoulder_pan_joint",   -1.0); break; }
            case KEYCODE_E:{ ROS_DEBUG("Shoulder lift +");  operateArm(hand_type, "shoulder_lift_joint",  +1.0); break; }
            case KEYCODE_D:{ ROS_DEBUG("Shoulder lift -");  operateArm(hand_type, "shoulder_lift_joint",  -1.0); break; }
            case KEYCODE_R:{ ROS_DEBUG("Upper arm roll +"); operateArm(hand_type, "upper_arm_roll_joint", +1.0); break; }
            case KEYCODE_F:{ ROS_DEBUG("Upper arm roll -"); operateArm(hand_type, "upper_arm_roll_joint", -1.0); break; }
            case KEYCODE_T:{ ROS_DEBUG("Elbow flex +");     operateArm(hand_type, "elbow_flex_joint",     +1.0); break; }
            case KEYCODE_G:{ ROS_DEBUG("Elbow flex -");     operateArm(hand_type, "elbow_flex_joint",     -1.0); break; }
            case KEYCODE_Y:{ ROS_DEBUG("Forearm roll +");   operateArm(hand_type, "forearm_roll_joint",   +1.0); break; }
            case KEYCODE_H:{ ROS_DEBUG("Forearm roll -");   operateArm(hand_type, "forearm_roll_joint",   -1.0); break; }
            case KEYCODE_U:{ ROS_DEBUG("Wrist flex +");     operateArm(hand_type, "wrist_flex_joint",     +1.0); break; }
            case KEYCODE_J:{ ROS_DEBUG("Wrist flex -");     operateArm(hand_type, "wrist_flex_joint",     -1.0); break; }
            case KEYCODE_I:{ ROS_DEBUG("Wrist roll +");     operateArm(hand_type, "wrist_roll_joint",     +1.0); break; }
            case KEYCODE_K:{ ROS_DEBUG("Wrist roll -");     operateArm(hand_type, "wrist_roll_joint",     -1.0); break; }

            case KEYCODE_O:{ ROS_DEBUG("Open Gripper");  operateGripper(hand_type, true);  break;}
            case KEYCODE_L:{ ROS_DEBUG("Close Gripper"); operateGripper(hand_type, false); break;}

            case KEYCODE_SPACE:{ ROS_DEBUG("Arm Stop"); operateArm(hand_type, "shoulder_pan_joint", 0.0); break; }
            case KEYCODE_Q:
            {
              is_arm_stop = true;
              showHelp();
              break;
            }
          }
        }
        break;
      }
    }
  }

  /////////////////////////////////////////////
  // cooked mode
  tcsetattr(kfd, TCSANOW, &cooked);
  /////////////////////////////////////////////

  return EXIT_SUCCESS;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_teleop_key");
  SIGVersePr2TeleopKey pr2_teleop_key;
  return pr2_teleop_key.run();
}
