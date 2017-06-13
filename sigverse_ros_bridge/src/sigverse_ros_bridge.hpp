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
#include <signal.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

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
#include <bsoncxx/types/value.hpp>

#include <boost/array.hpp>

#define TYPE_TWIST        "geometry_msgs/Twist"
#define TYPE_CAMERA_INFO  "sensor_msgs/CameraInfo"
#define TYPE_IMAGE        "sensor_msgs/Image"
#define TYPE_LASER_SCAN   "sensor_msgs/LaserScan"
#define TYPE_TF_LIST      "sigverse/TfList"

#define BUFFER_SIZE 25*1024*1024 //100MB

#define PORT 50001

class SIGVerseROSBridge
{
private:
	static pid_t gettid(void);

	static void rosSigintHandler(int sig);
	static bool checkReceivable( int fd );

	static void setVectorDouble(std::vector<double> &destVec, const bsoncxx::array::view &arrayView);
	static void setVectorFloat (std::vector<float>  &destVec, const bsoncxx::array::view &arrayView);

	template < size_t ArrayNum >
	static void setArrayDouble(boost::array<double, ArrayNum> &vec, const bsoncxx::array::view &arrayView);

	static void *receivingThread(void *param);

	static bool isRunning;

public:
	int run(int argc, char **argv);
};

#endif // SIGVERSE_ROS_BRIDGE_HPP
