#include "grasping_auto.hpp"

std::atomic<bool> SIGVerseTb3GraspingAuto::need_redraw_window_{false};

void SIGVerseTb3GraspingAuto::yolo_detection_callback(const yolo_msgs::msg::DetectionArray::SharedPtr detection_array)
{
  if (detection_array->detections.empty()) { return; }

  std::scoped_lock lock(yolo_mutex_);
  yolo_objects_ = *detection_array;   // deep copy
}

void SIGVerseTb3GraspingAuto::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state)
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


void SIGVerseTb3GraspingAuto::move_base(const double linear_x, const double angular_z)
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


void SIGVerseTb3GraspingAuto::move_arm(const std::string &name, const double position, const double current_pos)
{
  std::vector<std::string> names;
  names.push_back(name);

  std::vector<double> positions;
  positions.push_back(position);

  int duration_sec = calc_trajectory_duration(position, current_pos);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

  joint_trajectory.points.push_back(arm_joint_point);

  joint_trajectory.joint_names = names;
  joint_trajectory.points[0].positions = positions;
  joint_trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(duration_sec);

  pub_joint_trajectory_->publish(joint_trajectory);
}


void SIGVerseTb3GraspingAuto::move_gripper(const double position, const double current_pos)
{
  std::vector<std::string> joint_names {GRIP_JOINT_NAME, GRIP_JOINT_SUB_NAME};

  std::vector<double> positions;

  positions.push_back(position); // for grip_joint
  positions.push_back(position); // for grip_joint_sub

  int duration_sec = calc_trajectory_duration(position, current_pos);

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points.push_back(point);

  pub_joint_trajectory_->publish(joint_trajectory);
}


void SIGVerseTb3GraspingAuto::stop_joints(const int duration_sec)
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


bool SIGVerseTb3GraspingAuto::find_grasping_target(geometry_msgs::msg::Point &target_pos, const std::string &target_name)
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

  publish_debug_markers(frame_id, target_pos);

  return true;
}


bool SIGVerseTb3GraspingAuto::move_arm_toward_object(const std::string &target_name)
{
  auto logger = node_->get_logger();

  try
  {
    geometry_msgs::msg::Point target_pos;

    if(!find_grasping_target(target_pos, target_name)){ return false; }

    RCLCPP_INFO(logger, "Target Object=%s", target_name.c_str());

    // Gripper Open
    move_gripper(GRIP_MIN, grip_joint_pos_);

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
    RCLCPP_ERROR(logger, "move_arm_toward_object: Standard exception: %s",e.what());
  }
  catch (...) 
  {
    RCLCPP_ERROR(logger, "move_arm_toward_object: Unknown exception caught");
  }

  return true;
}


void SIGVerseTb3GraspingAuto::initialize_posture()
{
  // Rotate joint1
  move_arm(JOINT1_NAME, 0.0, joint1_pos_);

  // Rotate joint2, joint3, joint4
  move_arm(JOINT2_NAME, 0.0, joint2_pos_);
  move_arm(JOINT3_NAME, 0.0, joint3_pos_);
  move_arm(JOINT4_NAME, 0.0, joint4_pos_);
}


std::string SIGVerseTb3GraspingAuto::get_detected_objects_list()
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


void SIGVerseTb3GraspingAuto::show_detected_objects_list()
{
  int rows, cols;
  getmaxyx(stdscr, rows, cols);
  if (rows <= WINDOW_HEADER_HEIGHT + 6) { return; }

  resize_window();

  // Render the list at fixed positions just below the header
  mvprintw(WINDOW_HEADER_HEIGHT + 2, 0, "---------------------------");
  mvprintw(WINDOW_HEADER_HEIGHT + 3, 0, "Detected objects Info");
  mvprintw(WINDOW_HEADER_HEIGHT + 4, 0, "objects=%s", get_detected_objects_list().c_str());
  mvprintw(WINDOW_HEADER_HEIGHT + 5, 0, "---------------------------");

  move(WINDOW_HEADER_HEIGHT + 6, 0);

  refresh();
}

void SIGVerseTb3GraspingAuto::publish_debug_markers(const std::string& frame_id, const geometry_msgs::msg::Point& target_pos)
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

void SIGVerseTb3GraspingAuto::update_window_layout()
{
  int rows, cols;
  getmaxyx(stdscr, rows, cols);

  if (rows <= 2) { return; } // terminal unavailable
  if (rows <= header_height_ + 2) { header_height_ = rows - 1; }
  else                            { header_height_ = WINDOW_HEADER_HEIGHT + 2; }

  if (win_header_) 
  {
    wresize(win_header_, header_height_, cols);
    mvwin(win_header_, 0, 0);
    werase(stdscr);
  }
  else 
  {
    win_header_ = newwin(header_height_, cols, 0, 0);
    if (!win_header_) { return; }
    wrefresh(win_header_);
    move(header_height_, 0);
  }

  refresh();

  std::printf("\033[%d;%dr", header_height_ + 1, rows); // limit scrolling to [header+1 .. rows]
  std::fflush(stdout);
}

void SIGVerseTb3GraspingAuto::init_window() 
{
  setlocale(LC_ALL, "");
  initscr();
  refresh();
  update_window_layout();
}

void SIGVerseTb3GraspingAuto::resize_window() 
{
  update_window_layout();
  show_help();
}

void SIGVerseTb3GraspingAuto::shutdown_window() 
{
  std::printf("\033[r");
  std::fflush(stdout);

  if (win_header_) { delwin(win_header_); win_header_ = nullptr; }
  endwin();
}

void SIGVerseTb3GraspingAuto::show_help() 
{
  if (!win_header_){ return; }
  wclear(win_header_);
  box(win_header_, 0, 0);
  mvwprintw(win_header_, 1, 2, "arrow keys : Move");
  mvwprintw(win_header_, 2, 2, "Space: Stop");
  mvwprintw(win_header_, 3, 2, "w/s/d/a: Forward/Backward/Turn Right/Turn Left");
  mvwprintw(win_header_, 4, 2, "  1: Grasp %s", GRASPING_TARGET1_NAME.c_str());
  mvwprintw(win_header_, 5, 2, "  2: Grasp %s", GRASPING_TARGET2_NAME.c_str());
  mvwprintw(win_header_, 6, 2, "  3: Grasp %s", GRASPING_TARGET3_NAME.c_str());
  mvwprintw(win_header_, 7, 2, "  4: Grasp %s", GRASPING_TARGET4_NAME.c_str());
  mvwprintw(win_header_, 8, 2, "  5: Grasp %s", GRASPING_TARGET5_NAME.c_str());
  mvwprintw(win_header_, 9, 2, "o: Hand Open");
  mvwprintw(win_header_,10, 2, "l: Show Detected objects list");
  wrefresh(win_header_);
  move(header_height_, 0);
  refresh();
}

void SIGVerseTb3GraspingAuto::key_loop(int argc, char** argv)
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
    init_window();
    show_help();

    rclcpp::init(argc, argv);

    node_ = rclcpp::Node::make_shared("tb3_omc_grasping_auto");

    // Override the default ros sigint handler.
    signal(SIGINT, ros_sigint_handler);
    signal(SIGWINCH, [](int){ need_redraw_window_.store(true, std::memory_order_relaxed); });

    auto logger = node_->get_logger();

    rclcpp::Rate loop_rate(10);

    pub_base_twist_        = node_->create_publisher<geometry_msgs::msg::Twist>            ("/tb3/cmd_vel", 10);
    pub_joint_trajectory_  = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/tb3/joint_trajectory", 10);
    pub_debug_markers_     = node_->create_publisher<visualization_msgs::msg::MarkerArray> ("/tb3/debug_markers", 10);

    auto sub_joint_state     = node_->create_subscription<sensor_msgs::msg::JointState>      ("/tb3/joint_state",     10, std::bind(&SIGVerseTb3GraspingAuto::joint_state_callback, this, std::placeholders::_1));
    auto sub_yolo_detections = node_->create_subscription<yolo_msgs::msg::DetectionArray>    ("/yolo_objects/detections_3d", 10, std::bind(&SIGVerseTb3GraspingAuto::yolo_detection_callback, this, std::placeholders::_1));

    sleep(2);

    bool is_grasping = false;
    GraspingStage stage = GraspingStage::MoveArm;
    time_point<system_clock> latest_stage_time;

//    showHelp();

    while (rclcpp::ok())
    {
      if (need_redraw_window_.exchange(false)) { resize_window(); }

      if(!is_grasping)
      {
        if(can_receive_key(kfd))
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
              move_base(0.0, 0.0);
              stop_joints(1);
              break;
            }
            case 'w':
            case KEYCODE_UP:
            {
              RCLCPP_DEBUG(logger, "Go Forward");
              move_base(+LINEAR_VEL, 0.0);
              break;
            }
            case 's':
            case KEYCODE_DOWN:
            {
              RCLCPP_DEBUG(logger, "Go Back");
              move_base(-LINEAR_VEL, 0.0);
              break;
            }
            case 'd':
            case KEYCODE_RIGHT:
            {
              RCLCPP_DEBUG(logger, "Turn Right");
              move_base(0.0, -ANGULAR_VEL);
              break;
            }
            case 'a':
            case KEYCODE_LEFT:
            {
              RCLCPP_DEBUG(logger, "Turn Left");
              move_base(0.0, +ANGULAR_VEL);
              break;
            }
            case '1': { is_grasping = move_arm_toward_object(GRASPING_TARGET1_NAME); break; }
            case '2': { is_grasping = move_arm_toward_object(GRASPING_TARGET2_NAME); break; }
            case '3': { is_grasping = move_arm_toward_object(GRASPING_TARGET3_NAME); break; }
            case '4': { is_grasping = move_arm_toward_object(GRASPING_TARGET4_NAME); break; }
            case '5': { is_grasping = move_arm_toward_object(GRASPING_TARGET5_NAME); break; }
            case 'o':
            {
              RCLCPP_DEBUG(logger, "Hand Open");
              move_gripper(GRIP_MIN, grip_joint_pos_);
              break;
            }
            case 'l':
            {
              RCLCPP_DEBUG(logger, "Show Detected objects list");
              show_detected_objects_list();
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
            go_next(latest_stage_time, stage);
            break;
          }
          case WaitMoveArm:
          {
            if(is_waiting(latest_stage_time, 3000)){ break; }

            go_next(latest_stage_time, stage);
            break;
          }
          case Grasp:
          {
            RCLCPP_DEBUG(logger, "Hand Close");
            move_gripper(GRIP_MAX, grip_joint_pos_);

            go_next(latest_stage_time, stage);
            break;
          }
          case WaitGrasp:
          {
            if(is_waiting(latest_stage_time, 2000)){ break; }

            go_next(latest_stage_time, stage);
            break;
          }
          case UpArm:
          {
            RCLCPP_DEBUG(logger, "Up Arm");
            initialize_posture();

            go_next(latest_stage_time, stage);
            break;
          }
          case WaitUpArm:
          {
            if(is_waiting(latest_stage_time, 3000)){ break; }

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

  shutdown_window();

  return;
}


int main(int argc, char** argv)
{
  SIGVerseTb3GraspingAuto grasping_auto;

  grasping_auto.key_loop(argc, argv);

  return(EXIT_SUCCESS);
}

