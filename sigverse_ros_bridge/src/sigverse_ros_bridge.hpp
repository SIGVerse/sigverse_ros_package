#ifndef SIGVERSE_ROS_BRIDGE_HPP
#define SIGVERSE_ROS_BRIDGE_HPP

#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <map>

#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <bsoncxx/array/view.hpp>
#include <bsoncxx/builder/basic/sub_document.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/array.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/config/prelude.hpp>
#include <bsoncxx/document/value.hpp>
#include <bsoncxx/document/view.hpp>
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>
#include <bsoncxx/types/bson_value/value.hpp>

#include <boost/array.hpp>

#define TYPE_TWIST        "geometry_msgs/msg/Twist"
#define TYPE_CAMERA_INFO  "sensor_msgs/msg/CameraInfo"
#define TYPE_IMAGE        "sensor_msgs/msg/Image"
#define TYPE_LASER_SCAN   "sensor_msgs/msg/LaserScan"
#define TYPE_TIME_SYNC    "sigverse/TimeSync"
#define TYPE_TF_LIST      "sigverse/TfList"

#define BUFFER_SIZE 25*1024*1024 //100MB

#define DEFAULT_PORT 50001
#define DEFAULT_SYNC_TIME_MAX_NUM 1

class SIGVerseROSBridge
{
private:
  static pid_t get_tid(void);

  static bool check_receivable (int fd );

  static void set_vector_double(std::vector<double> &destVec, const bsoncxx::array::view &arrayView);
  static void set_vector_float (std::vector<float>  &destVec, const bsoncxx::array::view &arrayView);

  template < size_t ArrayNum >
  static void set_array_double(std::array<double, ArrayNum> &vec, const bsoncxx::array::view &arrayView);

  template < size_t ArrayNum >
  static void set_array_double(boost::array<double, ArrayNum> &vec, const bsoncxx::array::view &arrayView);

  static void *receiving_thread(void *param);

  static int  syncTimeCnt;
  static int  syncTimeMaxNum;

public:
  int run(int argc, char **argv);
};

#endif // SIGVERSE_ROS_BRIDGE_HPP
