#include "grasping_auto.hpp"

void SIGVerseTb3GraspingAuto::yoloDetectionCallback(const yolo_msgs::msg::DetectionArray::SharedPtr detection_array)
{
  if (detection_array->detections.empty()) { return; }

  std::scoped_lock lock(yolo_mutex_);
  yolo_objects_ = *detection_array;   // deep copy
}

void SIGVerseTb3GraspingAuto::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
{
  // Check Time Stamp
//  auto logger = node_->get_logger();
//  const auto &st = joint_state->header.stamp;
//  RCLCPP_INFO(logger, "JointState stamp: %d.%09u (sec.nsec)", st.sec, st.nanosec);

  for(size_t i=0; i<joint_state->name.size(); i++)
  {
    if(joint_state->name[i]==JOINT1_NAME)    { joint1_pos_     = joint_state->position[i]; continue; }
    if(joint_state->name[i]==JOINT2_NAME)    { joint2_pos_     = joint_state->position[i]; continue; }
    if(joint_state->name[i]==JOINT3_NAME)    { joint3_pos_     = joint_state->position[i]; continue; }
    if(joint_state->name[i]==JOINT4_NAME)    { joint4_pos_     = joint_state->position[i]; continue; }
    if(joint_state->name[i]==GRIP_JOINT_NAME){ grip_joint_pos_ = joint_state->position[i]; continue; }
  }
}


void SIGVerseTb3GraspingAuto::moveBase(const double linear_x, const double angular_z)
{
  geometry_msgs::msg::Twist twist;

  twist.linear.x  = linear_x;
  twist.linear.y  = 0.0;
  twist.linear.z  = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_z;

  pub_base_twist_->publish(twist);
}


void SIGVerseTb3GraspingAuto::moveArm(const std::string &name, const double position, const double current_pos)
{
  std::vector<std::string> names;
  names.push_back(name);

  std::vector<double> positions;
  positions.push_back(position);

  int duration_sec = calcTrajectoryDuration(position, current_pos);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

  joint_trajectory.points.push_back(arm_joint_point);

  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(duration_sec);

  pub_joint_trajectory_->publish(joint_trajectory);
}


void SIGVerseTb3GraspingAuto::moveGripper(const double position, const double current_pos)
{
  std::vector<std::string> joint_names {GRIP_JOINT_NAME, GRIP_JOINT_SUB_NAME};

  std::vector<double> positions;

  positions.push_back(position); // for grip_joint
  positions.push_back(position); // for grip_joint_sub

  int duration_sec = calcTrajectoryDuration(position, current_pos);

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);

  pub_joint_trajectory_->publish(joint_trajectory);
}


void SIGVerseTb3GraspingAuto::stopJoints(const int duration_sec)
{
  std::vector<std::string> names {JOINT1_NAME, JOINT2_NAME, JOINT3_NAME, JOINT4_NAME};

  std::vector<double> positions;
  positions.push_back(joint1_pos_);
  positions.push_back(joint2_pos_);
  positions.push_back(joint3_pos_);
  positions.push_back(joint4_pos_);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

  joint_trajectory.points.push_back(arm_joint_point);

  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(duration_sec);

  pub_joint_trajectory_->publish(joint_trajectory);
}


bool SIGVerseTb3GraspingAuto::findGraspingTarget(geometry_msgs::msg::Point &target_pos, const std::string &target_name)
{
  std::scoped_lock lock(yolo_mutex_);
  if (!yolo_objects_ || yolo_objects_->detections.empty()){ return false; }

  double nearest_distance = std::numeric_limits<double>::infinity();
  std::optional<geometry_msgs::msg::Point> nearest_pos;
  std::string frame_id;

  for (const auto &detection : yolo_objects_->detections) 
  {
    if (detection.class_name != target_name){ continue; }

    // NOTE: Assumes bbox3d.center.position is already expressed in target_frame (e.g., base_link).
    const auto &pos = detection.bbox3d.center.position;
    const double distance = std::sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z);

    if (distance < nearest_distance) 
    {
      nearest_distance = distance;
      nearest_pos = pos;
      frame_id = detection.bbox3d.frame_id;
    }
  }

  if (nearest_pos==std::nullopt){ return false; }

  target_pos = nearest_pos.value();

  publishDebugMarkers(frame_id, target_pos);

  return true;
}


bool SIGVerseTb3GraspingAuto::moveArmTowardObject(const std::string &target_name)
{
  auto logger = node_->get_logger();

  try
  {
    geometry_msgs::msg::Point target_pos;

    if(!findGraspingTarget(target_pos, target_name)){ return false; }

    RCLCPP_INFO(logger, "Target Object=%s", target_name.c_str());

    // Gripper Open
    moveGripper(GRIP_MIN, grip_joint_pos_);

    RCLCPP_INFO(logger, "MoveIt -START-");

    moveit::planning_interface::MoveGroupInterface arm(node_, "arm");
    arm.setPoseReferenceFrame("base_link");
    arm.setEndEffectorLink("end_effector_link");
    arm.setGoalPositionTolerance(0.01); //[m]
    arm.setGoalOrientationTolerance(0.05); //[rad]

//    puts(("target_pos.x= " + std::to_string(target_pos.x)).c_str());
//    puts(("target_pos.y= " + std::to_string(target_pos.y)).c_str());
//    puts(("target_pos.z= " + std::to_string(target_pos.z)).c_str());

    arm.setPositionTarget(target_pos.x, target_pos.y, target_pos.z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode result = arm.plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "Plan succeeded, executing...");      
      arm.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "Plan failed, cannot execute.");
    }

    RCLCPP_INFO(logger, "MoveIt -END-");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger, "moveArmTowardObject: Standard exception: %s",e.what());
  }
  catch (...) 
  {
    RCLCPP_ERROR(logger, "moveArmTowardObject: Unknown exception caught");
  }

  return true;
}


void SIGVerseTb3GraspingAuto::initializePosture()
{
  // Rotate joint1
  moveArm(JOINT1_NAME, 0.0, joint1_pos_);

  // Rotate joint2, joint3, joint4
  moveArm(JOINT2_NAME, 0.0, joint2_pos_);
  moveArm(JOINT3_NAME, 0.0, joint3_pos_);
  moveArm(JOINT4_NAME, 0.0, joint4_pos_);
}


std::string SIGVerseTb3GraspingAuto::getDetectedObjectsList()
{
  std::scoped_lock lock(yolo_mutex_);
  if (!yolo_objects_ || yolo_objects_->detections.empty()){ return ""; }

  std::string detected_objects = "";

  for (const auto &detection : yolo_objects_->detections)
  {
    detected_objects += "/" + detection.class_name;
  }

  return detected_objects;
}


void SIGVerseTb3GraspingAuto::showDetectedObjectsList()
{
  puts("\n");
  puts("---------------------------");
  puts("Detected objects Info");
  puts(("objects=" + getDetectedObjectsList()).c_str());
  puts("---------------------------");
}

void SIGVerseTb3GraspingAuto::publishDebugMarkers(const std::string& frame_id, const geometry_msgs::msg::Point& target_pos)
{
  visualization_msgs::msg::MarkerArray markerArray;
  const auto stamp = rclcpp::Time(0);

  visualization_msgs::msg::Marker target;
  target.header.frame_id = frame_id;
  target.header.stamp    = stamp;
  target.ns   = "target";
  target.id   = 0;  // Keep constant to overwrite
  target.type = visualization_msgs::msg::Marker::SPHERE;
  target.action = visualization_msgs::msg::Marker::ADD;
  target.scale = []{ geometry_msgs::msg::Vector3 v; v.x=0.1; v.y=0.1; v.z=0.1; return v; }();
  target.color = []{ std_msgs::msg::ColorRGBA c; c.r=1.0f; c.g=0.0f; c.b=0.0f; c.a=1.0f; return c; }(); // red
  target.pose.orientation.w = 1.0;
  target.pose.position = target_pos;
  markerArray.markers.push_back(target);

  pub_debug_markers_->publish(markerArray);
}

void SIGVerseTb3GraspingAuto::showHelp()
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

void SIGVerseTb3GraspingAuto::keyLoop(int argc, char** argv)
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
    rclcpp::init(argc, argv);

    node_ = rclcpp::Node::make_shared("tb3_omc_grasping_auto");

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, rosSigintHandler);

    auto logger = node_->get_logger();

    rclcpp::Rate loop_rate(10);

    pub_base_twist_        = node_->create_publisher<geometry_msgs::msg::Twist>            ("/tb3/cmd_vel", 10);
    pub_joint_trajectory_  = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/tb3/joint_trajectory", 10);
    pub_debug_markers_     = node_->create_publisher<visualization_msgs::msg::MarkerArray> ("/tb3/debug_markers", 10);

    auto sub_joint_state     = node_->create_subscription<sensor_msgs::msg::JointState>      ("/tb3/joint_state",     10, std::bind(&SIGVerseTb3GraspingAuto::jointStateCallback, this, std::placeholders::_1));
    auto sub_yolo_detections = node_->create_subscription<yolo_msgs::msg::DetectionArray>    ("/yolo_objects/detections_3d", 10, std::bind(&SIGVerseTb3GraspingAuto::yoloDetectionCallback, this, std::placeholders::_1));

    sleep(2);

    bool is_grasping = false;
    GraspingStage stage = GraspingStage::MoveArm;
    time_point<system_clock> latest_stage_time;

    showHelp();

    while (rclcpp::ok())
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
            case ' ':
            {
              RCLCPP_DEBUG(logger, "Stop");
              moveBase(0.0, 0.0);
              stopJoints(1);
              break;
            }
            case 'w':
            case KEYCODE_UP:
            {
              RCLCPP_DEBUG(logger, "Go Forward");
              moveBase(+LINEAR_VEL, 0.0);
              break;
            }
            case 's':
            case KEYCODE_DOWN:
            {
              RCLCPP_DEBUG(logger, "Go Back");
              moveBase(-LINEAR_VEL, 0.0);
              break;
            }
            case 'd':
            case KEYCODE_RIGHT:
            {
              RCLCPP_DEBUG(logger, "Turn Right");
              moveBase(0.0, -ANGULAR_VEL);
              break;
            }
            case 'a':
            case KEYCODE_LEFT:
            {
              RCLCPP_DEBUG(logger, "Turn Left");
              moveBase(0.0, +ANGULAR_VEL);
              break;
            }
            case '1': { is_grasping = moveArmTowardObject(GRASPING_TARGET1_NAME); break; }
            case '2': { is_grasping = moveArmTowardObject(GRASPING_TARGET2_NAME); break; }
            case '3': { is_grasping = moveArmTowardObject(GRASPING_TARGET3_NAME); break; }
            case '4': { is_grasping = moveArmTowardObject(GRASPING_TARGET4_NAME); break; }
            case '5': { is_grasping = moveArmTowardObject(GRASPING_TARGET5_NAME); break; }
            case 'o':
            {
              RCLCPP_DEBUG(logger, "Hand Open");
              moveGripper(GRIP_MIN, grip_joint_pos_);
              break;
            }
            case 'l':
            {
              RCLCPP_DEBUG(logger, "Show Detected objects list");
              showDetectedObjectsList();
              break;
            }
            case 'h':
            {
              RCLCPP_DEBUG(logger, "Show Help");
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
            RCLCPP_DEBUG(logger, "Hand Close");
            moveGripper(GRIP_MAX, grip_joint_pos_);

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
            RCLCPP_DEBUG(logger, "Up Arm");
            initializePosture();

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

      rclcpp::spin_some(node_);

//      puts("rclcpp::spin rclcpp::spin rclcpp::spin rclcpp::spin rclcpp::spin");

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
  SIGVerseTb3GraspingAuto grasping_auto;

  grasping_auto.keyLoop(argc, argv);

  return(EXIT_SUCCESS);
}

