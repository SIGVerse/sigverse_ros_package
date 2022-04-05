#include "grasping_auto.hpp"

void SIGVerseTb3OpenManipulatorGraspingAuto::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
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


void SIGVerseTb3OpenManipulatorGraspingAuto::rgbCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info)
{
  rgb_camera_height_ = camera_info->height;
  rgb_camera_width_  = camera_info->width;
}


void SIGVerseTb3OpenManipulatorGraspingAuto::boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bounding_boxes)
{
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(system_clock::now() - latest_time_of_bounding_boxes_).count();

  if(duration < OBJECTS_INFO_UPDATING_INTERVAL){ return; }

  latest_time_of_bounding_boxes_ = system_clock::now();

//  ROS_INFO("boundingBoxesCallback size=%d", (int)bounding_boxes->boundingBoxes.size());

  int data_count = std::min<int>(bounding_boxes->bounding_boxes.size(), MAX_OBJECTS_NUM);

  bounding_boxes_data_.bounding_boxes.resize(data_count);

  for(int i=0; i<data_count; i++)
  {
    bounding_boxes_data_.bounding_boxes[i].Class       = bounding_boxes->bounding_boxes[i].Class;
    bounding_boxes_data_.bounding_boxes[i].probability = bounding_boxes->bounding_boxes[i].probability;
    bounding_boxes_data_.bounding_boxes[i].xmin        = bounding_boxes->bounding_boxes[i].xmin;
    bounding_boxes_data_.bounding_boxes[i].ymin        = bounding_boxes->bounding_boxes[i].ymin;
    bounding_boxes_data_.bounding_boxes[i].xmax        = bounding_boxes->bounding_boxes[i].xmax;
    bounding_boxes_data_.bounding_boxes[i].ymax        = bounding_boxes->bounding_boxes[i].ymax;
  }
}


void SIGVerseTb3OpenManipulatorGraspingAuto::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud)
{
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(system_clock::now() - latest_time_of_point_cloud_).count();

  if(duration < OBJECTS_INFO_UPDATING_INTERVAL){ return; }

  latest_time_of_point_cloud_ = system_clock::now();

//  ROS_INFO("pointCloudCallback row_step=%d", (int)point_cloud->row_step);

  point_cloud_data_.header.seq        = point_cloud->header.seq;
  point_cloud_data_.header.stamp.sec  = point_cloud->header.stamp.sec;
  point_cloud_data_.header.stamp.nsec = point_cloud->header.stamp.nsec;
  point_cloud_data_.header.frame_id   = point_cloud->header.frame_id;

  point_cloud_data_.height = point_cloud->height;
  point_cloud_data_.width  = point_cloud->width;

  point_cloud_data_.fields.resize(point_cloud->fields.size());

  for(int i=0; i<point_cloud->fields.size(); i++)
  {
    point_cloud_data_.fields[i].name     = point_cloud->fields[i].name;
    point_cloud_data_.fields[i].offset   = point_cloud->fields[i].offset;
    point_cloud_data_.fields[i].datatype = point_cloud->fields[i].datatype;
    point_cloud_data_.fields[i].count    = point_cloud->fields[i].count;
  }

  point_cloud_data_.is_bigendian = point_cloud->is_bigendian;
  point_cloud_data_.point_step   = point_cloud->point_step;
  point_cloud_data_.row_step     = point_cloud->row_step;

  int data_count = point_cloud->row_step * point_cloud->height;

  point_cloud_data_.data.resize(data_count);
  std::memcpy(&point_cloud_data_.data[0], &point_cloud->data[0], data_count * sizeof(uint8_t));

  point_cloud_data_.is_dense = point_cloud->is_dense;
}


void SIGVerseTb3OpenManipulatorGraspingAuto::moveBase(ros::Publisher &publisher, const double linear_x, const double angular_z)
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


void SIGVerseTb3OpenManipulatorGraspingAuto::moveArm(ros::Publisher &publisher, const std::string &name, const double position, const double current_pos)
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


void SIGVerseTb3OpenManipulatorGraspingAuto::moveHand(ros::Publisher &publisher, const double position, const double current_pos)
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


void SIGVerseTb3OpenManipulatorGraspingAuto::stopJoints(ros::Publisher &publisher, const int duration_sec)
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


bool SIGVerseTb3OpenManipulatorGraspingAuto::findGraspingTarget(geometry_msgs::Vector3 &point_cloud_pos, const std::string &target_name)
{
  for(int i=0; i<bounding_boxes_data_.bounding_boxes.size(); i++)
  {
    if(bounding_boxes_data_.bounding_boxes[i].Class == target_name &&
       bounding_boxes_data_.bounding_boxes[i].probability > PROBABILITY_THRESHOLD)
    {
      puts(("Found the target. name= " + target_name).c_str());

      int center_x = (bounding_boxes_data_.bounding_boxes[i].xmax + bounding_boxes_data_.bounding_boxes[i].xmin) / 2;
      int center_y = (bounding_boxes_data_.bounding_boxes[i].ymax + bounding_boxes_data_.bounding_boxes[i].ymin) / 2;

//      puts(("x=" + std::to_string(center_x) + ", y=" + std::to_string(center_y)).c_str());

      int point_cloud_screen_x = clamp<int>((int)((float)center_x * point_cloud_data_.width  / rgb_camera_width_),  0, point_cloud_data_.width-1);
      int point_cloud_screen_y = clamp<int>((int)((float)center_y * point_cloud_data_.height / rgb_camera_height_), 0, point_cloud_data_.height-1);

      // the center
      bool is_succeeded = get3dPositionFromScreenPosition(point_cloud_pos, point_cloud_data_, point_cloud_screen_x, point_cloud_screen_y);

//      puts(("x=" + std::to_string(point_cloud_pos.x) + ", y=" + std::to_string(point_cloud_pos.y) + ", z=" + std::to_string(point_cloud_pos.z)).c_str());

      if(is_succeeded) { return true; }

      // Around the center (1/4)
      int play_x = (bounding_boxes_data_.bounding_boxes[i].xmax - center_x) / 4;
      int play_y = (bounding_boxes_data_.bounding_boxes[i].ymax - center_y) / 4;

      for(int yi=-play_y; yi<=+play_y; yi+=play_y)
      {
        for(int xi=-play_x; xi<=+play_x; xi+=play_x)
        {
          is_succeeded = get3dPositionFromScreenPosition(point_cloud_pos, point_cloud_data_, point_cloud_screen_x + xi, point_cloud_screen_y + yi);
          if(is_succeeded) { return true; }
        }
      }

      // Around the center (1/2)
      play_x = (bounding_boxes_data_.bounding_boxes[i].xmax - center_x) / 2;
      play_y = (bounding_boxes_data_.bounding_boxes[i].ymax - center_y) / 2;

      for(int yi=-play_y; yi<=+play_y; yi+=play_y)
      {
        for(int xi=-play_x; xi<=+play_x; xi+=play_x)
        {
          is_succeeded = get3dPositionFromScreenPosition(point_cloud_pos, point_cloud_data_, point_cloud_screen_x + xi, point_cloud_screen_y + yi);
          if(is_succeeded) { return true; }
        }
      }

      puts("Failed to get point cloud data.");
      return false;
    }
  }

  puts(("Couldn't find " + target_name + ". Or low probability.").c_str());
  puts(("objects=" + getDetectedObjectsList()).c_str());

  return false;
}


bool SIGVerseTb3OpenManipulatorGraspingAuto::moveArmTowardObject(tf::TransformBroadcaster &tf_broadcaster, tf::TransformListener &tf_listener, ros::Publisher &pub_joint_traj, const std::string &target_name)
{
  geometry_msgs::Vector3 target_pos;

  if(!findGraspingTarget(target_pos, target_name)){ return false; }

  tf::Transform target_transform;

  target_transform.setOrigin( tf::Vector3(target_pos.x, target_pos.y, target_pos.z) );
  target_transform.setRotation( tf::Quaternion::getIdentity() );

  tf_broadcaster.sendTransform(tf::StampedTransform(target_transform, ros::Time::now(), CAMERA_DEPTH_OPTICAL_FRAME_NAME, TARGET_NAME));

  tf::StampedTransform transform_link1_to_target, transform_link3_to_target;
  tf::StampedTransform transform_link3_to_link4,  transform_link4_to_link5;

  try
  {
    tf_listener.waitForTransform(LINK1_NAME, TARGET_NAME, ros::Time(0), ros::Duration(0.3) );

    tf_listener.lookupTransform (LINK1_NAME, TARGET_NAME, ros::Time(0), transform_link1_to_target);
    tf_listener.lookupTransform (LINK3_NAME, TARGET_NAME, ros::Time(0), transform_link3_to_target);
    tf_listener.lookupTransform (LINK3_NAME, LINK4_NAME,  ros::Time(0), transform_link3_to_link4);
    tf_listener.lookupTransform (LINK4_NAME, LINK5_NAME,  ros::Time(0), transform_link4_to_link5);
  }
  catch (tf::TransformException &ex)
  {
    puts(("Couldn't lookup the transform of TF."));
    ROS_ERROR("%s",ex.what());
    return false;
  }

  tf::Vector3 vec_link3_to_target = transform_link3_to_target.getOrigin();
  tf::Vector3 vec_link3_to_link4  = transform_link3_to_link4.getOrigin();
  tf::Vector3 vec_link4_to_link5  = transform_link4_to_link5.getOrigin();

  double len_link3_target = std::sqrt(std::pow(vec_link3_to_target.x(),2) + std::pow(vec_link3_to_target.z(),2));
  double len_link3_link4  = std::sqrt(std::pow(vec_link3_to_link4.x(),2)  + std::pow(vec_link3_to_link4.z(),2));
  double len_link4_link5  = std::sqrt(std::pow(vec_link4_to_link5.x(),2)  + std::pow(vec_link4_to_link5.z(),2));

  double distance = len_link3_target - 0.12; // Shorten the distance 12cm.

  if(distance > len_link3_link4 + len_link4_link5)
  {
    puts(("The target is too far. distance="+std::to_string(distance)).c_str());
    return false;
  }

  ROS_DEBUG("Grasp %s", target_name.c_str());

  // Rotate joint1
  tf::Vector3 vec_link1_to_target = transform_link1_to_target.getOrigin();

//  puts(("link1 to target x=" + std::to_string(vec_link1_to_target.x()) + ", y=" + std::to_string(vec_link1_to_target.y()) + ", z=" + std::to_string(vec_link1_to_target.z())).c_str());

  double joint1_angle = std::atan2(vec_link1_to_target.y(), vec_link1_to_target.x());

//  puts(("joint1_angle="+std::to_string(joint1_angle)).c_str());

  moveArm(pub_joint_traj, JOINT1_NAME, joint1_angle, joint1_pos_);

  // Rotate joint2, joint3, joint4
  double joint2_angle = M_PI/2.0 - std::acos((std::pow(len_link3_link4,2) + std::pow(distance,2) - std::pow(len_link4_link5,2)) / (2.0 * len_link3_link4 * distance));
  double joint3_angle = M_PI/2.0 - std::acos((std::pow(len_link3_link4,2) + std::pow(len_link4_link5,2) - std::pow(distance,2)) / (2.0 * len_link3_link4 * len_link4_link5));

//  puts(("joint2_angle="+std::to_string(joint2_angle)).c_str());
//  puts(("joint3_angle="+std::to_string(joint3_angle)).c_str());

  moveArm(pub_joint_traj, JOINT2_NAME, joint2_angle, joint2_pos_);
  moveArm(pub_joint_traj, JOINT3_NAME, joint3_angle, joint3_pos_);

  double joint4_angle = -(joint2_angle + joint3_angle) + 0.5; // 0.5 means adjustment. Rotate a little downward.

  moveArm(pub_joint_traj, JOINT4_NAME, joint4_angle, joint4_pos_);

  // Hand Open
  moveHand(pub_joint_traj, GRIP_MIN, grip_joint_pos_);

  return true;
}


void SIGVerseTb3OpenManipulatorGraspingAuto::initializePosture(ros::Publisher &pub_joint_traj)
{
  // Rotate joint1
  moveArm(pub_joint_traj, JOINT1_NAME, 0.0, joint1_pos_);

  // Rotate joint2, joint3, joint4
  moveArm(pub_joint_traj, JOINT2_NAME, 0.0, joint2_pos_);
  moveArm(pub_joint_traj, JOINT3_NAME, 0.0, joint3_pos_);
  moveArm(pub_joint_traj, JOINT4_NAME, 0.0, joint4_pos_);
}


std::string SIGVerseTb3OpenManipulatorGraspingAuto::getDetectedObjectsList()
{
  std::string detected_objects_ = "";

  for(int i=0; i<bounding_boxes_data_.bounding_boxes.size(); i++)
  {
    detected_objects_
      += bounding_boxes_data_.bounding_boxes[i].Class + ": "
      + std::to_string((int)std::floor(bounding_boxes_data_.bounding_boxes[i].probability * 100)) + "%   ";
  }

  return detected_objects_;
}


void SIGVerseTb3OpenManipulatorGraspingAuto::showDetectedObjectsList()
{
  puts("\n");
  puts("---------------------------");
  puts("Detected objects Info");
  puts(("objects=" + getDetectedObjectsList()).c_str());
  puts("---------------------------");
}

void SIGVerseTb3OpenManipulatorGraspingAuto::showHelp()
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
  puts(("1: Grasp " + GRASPING_TARGET1_NAME).c_str());
  puts(("2: Grasp " + GRASPING_TARGET2_NAME).c_str());
  puts(("3: Grasp " + GRASPING_TARGET3_NAME).c_str());
  puts(("4: Grasp " + GRASPING_TARGET4_NAME).c_str());
  puts(("5: Grasp " + GRASPING_TARGET5_NAME).c_str());
  puts("---------------------------");
  puts("o: Hand Open");
  puts("---------------------------");
  puts("l: Show Detected objects list");
  puts("h: Show help");
}


void SIGVerseTb3OpenManipulatorGraspingAuto::keyLoop(int argc, char** argv)
{
  char c;
  int  ret;
  char buf[1024];

  /////////////////////////////////////////////
  int kfd = 0;
  struct termios cooked;

  // get the console in raw mode
  struct termios raw;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
  /////////////////////////////////////////////

  try
  {
    ros::init(argc, argv, "tb3_omc_teleop_key", ros::init_options::NoSigintHandler);

    ros::NodeHandle node_handle;

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, rosSigintHandler);

    ros::Rate loop_rate(10);

    std::string sub_joint_state_topic_name;
    std::string pub_base_twist_topic_name;
    std::string pub_joint_trajectory_topic_name;
    std::string sub_rgb_camera_info_topic_name;
    std::string sub_point_cloud_topic_name;
    std::string sub_bounding_boxes_topic_name;

    node_handle.param<std::string>("grasping_auto/sub_joint_state_topic_name",      sub_joint_state_topic_name,      "/tb3omc/joint_state");
    node_handle.param<std::string>("grasping_auto/pub_twist_topic_name",            pub_base_twist_topic_name,       "/tb3omc/cmd_vel");
    node_handle.param<std::string>("grasping_auto/pub_joint_trajectory_topic_name", pub_joint_trajectory_topic_name, "/tb3omc/joint_trajectory");
    node_handle.param<std::string>("grasping_auto/sub_rgb_camera_info_topic_name",  sub_rgb_camera_info_topic_name,  "/camera/rgb/camera_info");
    node_handle.param<std::string>("grasping_auto/sub_point_cloud_topic_name",      sub_point_cloud_topic_name,      "/camera/depth/points");
    node_handle.param<std::string>("grasping_auto/sub_bounding_boxes_topic_name",   sub_bounding_boxes_topic_name,   "/darknet_ros/bounding_boxes");

    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener tf_listener;

    ros::Subscriber sub_joint_state = node_handle.subscribe(sub_joint_state_topic_name, 10, &SIGVerseTb3OpenManipulatorGraspingAuto::jointStateCallback, this);
    ros::Publisher pub_base_twist = node_handle.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
    ros::Publisher pub_joint_traj = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_joint_trajectory_topic_name, 10);

    ros::Subscriber sub_rgb_camera_info = node_handle.subscribe(sub_rgb_camera_info_topic_name, 10, &SIGVerseTb3OpenManipulatorGraspingAuto::rgbCameraInfoCallback, this);
    ros::Subscriber sub_point_cloud     = node_handle.subscribe(sub_point_cloud_topic_name,     10, &SIGVerseTb3OpenManipulatorGraspingAuto::pointCloudCallback, this);
    ros::Subscriber sub_bounding_boxes  = node_handle.subscribe(sub_bounding_boxes_topic_name,  10, &SIGVerseTb3OpenManipulatorGraspingAuto::boundingBoxesCallback, this);

    sleep(2);

    bool is_grasping = false;
    GraspingStage stage = GraspingStage::MoveArm;
    time_point<system_clock> latest_stage_time;

    showHelp();

    while (ros::ok())
    {
      if(!is_grasping)
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
              stopJoints(pub_joint_traj, 1);
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
            case KEY_1:
            {
              is_grasping = moveArmTowardObject(tf_broadcaster, tf_listener, pub_joint_traj, GRASPING_TARGET1_NAME);
              break;
            }
            case KEY_2:
            {
              is_grasping = moveArmTowardObject(tf_broadcaster, tf_listener, pub_joint_traj, GRASPING_TARGET2_NAME);
              break;
            }
            case KEY_3:
            {
              is_grasping = moveArmTowardObject(tf_broadcaster, tf_listener, pub_joint_traj, GRASPING_TARGET3_NAME);
              break;
            }
            case KEY_4:
            {
              is_grasping = moveArmTowardObject(tf_broadcaster, tf_listener, pub_joint_traj, GRASPING_TARGET4_NAME);
              break;
            }
            case KEY_5:
            {
              is_grasping = moveArmTowardObject(tf_broadcaster, tf_listener, pub_joint_traj, GRASPING_TARGET5_NAME);
              break;
            }
            case KEY_O:
            {
              ROS_DEBUG("Hand Open");
              moveHand(pub_joint_traj, GRIP_MIN, grip_joint_pos_);
              break;
            }
            case KEY_L:
            {
              ROS_DEBUG("Show Detected objects list");
              showDetectedObjectsList();
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
      }
      else
      {
        switch(stage)
        {
          case MoveArm:
          {
            goNext(latest_stage_time, stage);
            break;
          }
          case WaitMoveArm:
          {
            if(isWaiting(latest_stage_time, 3000)){ break; }

            goNext(latest_stage_time, stage);
            break;
          }
          case Grasp:
          {
            ROS_DEBUG("Hand Close");
            moveHand(pub_joint_traj, GRIP_MAX, grip_joint_pos_);

            goNext(latest_stage_time, stage);
            break;
          }
          case WaitGrasp:
          {
            if(isWaiting(latest_stage_time, 2000)){ break; }

            goNext(latest_stage_time, stage);
            break;
          }
          case UpArm:
          {
            ROS_DEBUG("Up Arm");
            initializePosture(pub_joint_traj);

            goNext(latest_stage_time, stage);
            break;
          }
          case WaitUpArm:
          {
            if(isWaiting(latest_stage_time, 3000)){ break; }

            stage = GraspingStage::MoveArm;
            is_grasping = false;
            break;
          }
        }
      }

      ros::spinOnce();

      loop_rate.sleep();
    }
  }
  catch(...)
  {
    puts("An exception occurred!");
  }

  /////////////////////////////////////////////
  // cooked mode
  tcsetattr(kfd, TCSANOW, &cooked);
  /////////////////////////////////////////////

  return;
}


int main(int argc, char** argv)
{
  SIGVerseTb3OpenManipulatorGraspingAuto grasping_auto;

  grasping_auto.keyLoop(argc, argv);

  return(EXIT_SUCCESS);
}

