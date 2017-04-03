
#include "sigverse_ros_bridge.hpp"

bool SIGVerseROSBridge::isInterrupted = false;

pid_t SIGVerseROSBridge::gettid(void)
{
	return syscall(SYS_gettid);
}

void SIGVerseROSBridge::interruptEventHandler(int sig)
{
	isInterrupted = true;
}

bool SIGVerseROSBridge::checkReceivable( int fd )
{
	fd_set fdset;
	int ret;
	struct timeval timeout;
	FD_ZERO( &fdset );
	FD_SET( fd , &fdset );

	// timeout is 1 sec
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;

	ret = select( fd+1 , &fdset , NULL , NULL , &timeout );

	return ret == 1;
}


void SIGVerseROSBridge::setVectorDouble(std::vector<double> &destVec, const bsoncxx::array::view &arrayView)
{
	int i = 0;

	for(auto itr = arrayView.cbegin(); itr != arrayView.cend(); ++itr)
	{
		destVec[i++] = (*itr).get_double();
	}
}

void SIGVerseROSBridge::setVectorFloat(std::vector<float> &destVec, const bsoncxx::array::view &arrayView)
{
	int i = 0;

	for(auto itr = arrayView.cbegin(); itr != arrayView.cend(); ++itr)
	{
		destVec[i++] = (float)((*itr).get_double());
	}
}

template < size_t ArrayNum >
void SIGVerseROSBridge::setArrayDouble(boost::array<double, ArrayNum> &destArray, const bsoncxx::array::view &arrayView)
{
	int i = 0;

	for(auto itr = arrayView.cbegin(); itr != arrayView.cend(); ++itr)
	{
		destArray[i++] = (*itr).get_double();
	}
}



void * SIGVerseROSBridge::receivingThread(void *param)
{
	signal(SIGINT, interruptEventHandler);

	int dstSocket = *((int *)param);

	int dummyArgc;
	char **dummyArgv;

	char buf[BUFFER_SIZE];
	long int totalReceivedSize;

	std::map<std::string, ros::Publisher> publisherMap;

	std::cout << "Socket open. tid=" << gettid() << std::endl;

	// Initialize ROS
	ros::init(dummyArgc, dummyArgv, "sigverse_ros_bridge_"+std::to_string(gettid()));

	ros::NodeHandle rosNodeHandle;
//	ros::Rate loop_rate(100);

	while(ros::ok())
	{
		// Get total BSON data size
		totalReceivedSize = 0;

		if(!checkReceivable(dstSocket)){ continue; }

		char bufHeader[4];

		long int numRcv = read(dstSocket, bufHeader, sizeof(4));

		if(numRcv == 0)
		{
			close(dstSocket);
			std::cout << "Socket closed. tid=" << gettid() << std::endl;
			break;
		}
		if(numRcv == -1)
		{
			close(dstSocket);
			std::cout << "Socket error. tid=" << gettid() << std::endl;
			break;
		}
		if(numRcv < 4)
		{
			close(dstSocket);
			std::cout << "Can not get data size... tid=" << gettid() << std::endl;
			break;
		}
		totalReceivedSize += 4;

		int32_t msgSize;
		memcpy(&msgSize, &bufHeader, sizeof(int32_t));
		memcpy(&buf[0] , &bufHeader, sizeof(int32_t));

		if(msgSize > BUFFER_SIZE)
		{
			close(dstSocket);
			std::cout << "Data size is too big. tid=" << gettid() << std::endl;
			break;
		}

//		std::cout << "msg size=" << msgSize << std::endl;

		// Get BSON data
		while(msgSize!=totalReceivedSize)
		{
			size_t unreceivedSize = msgSize - (size_t)totalReceivedSize;

			if(!checkReceivable(dstSocket)){ break; }

			long int receivedSize = read(dstSocket, &(buf[totalReceivedSize]), unreceivedSize);

			totalReceivedSize += receivedSize;

//			std::cout << "receivedSize=" << receivedSize << std::endl;
		}

		if(msgSize!=totalReceivedSize)
		{
			std::cout << "msgSize!=totalReceivedSize ?????? tid=" << gettid() << std::endl;
			continue;
		};


		bsoncxx::document::view bsonView((const uint8_t*)buf, (std::size_t)msgSize);

		bsoncxx::builder::core bsonCore(false);

		bsonCore.concatenate(bsonView);

		bsoncxx::builder::basic::sub_document bsonSubdocument(&bsonCore);

		std::string opValue	   = bsonView["op"]   .get_utf8().value.to_string();
		std::string topicValue = bsonView["topic"].get_utf8().value.to_string();
		std::string typeValue  = bsonView["type"] .get_utf8().value.to_string();
//		std::cout << "op:" << opValue << std::endl;
//		std::cout << "tp:" << topicValue << std::endl;

		// Advertise
		if(publisherMap.count(topicValue)==0 && typeValue!=TYPE_TF_LIST)
		{
			ros::Publisher publisher;

			if(typeValue==TYPE_TWIST)
			{
				publisher = rosNodeHandle.advertise<geometry_msgs::Twist>(topicValue, 1000);
			}
			else if(typeValue==TYPE_CAMERA_INFO)
			{
				publisher = rosNodeHandle.advertise<sensor_msgs::CameraInfo>(topicValue, 10);
			}
			else if(typeValue==TYPE_IMAGE)
			{
				publisher = rosNodeHandle.advertise<sensor_msgs::Image>(topicValue, 10);
			}
			else if(typeValue==TYPE_LASER_SCAN)
			{
				publisher = rosNodeHandle.advertise<sensor_msgs::LaserScan>(topicValue, 10);
			}
			else
			{
				std::cout << "Not compatible message type! :" << typeValue << std::endl;
				continue;
			}

			std::cout << "Advertised " << topicValue << std::endl;
			publisherMap[topicValue] = publisher;
		}

		// Publish
		// Twist
		if(typeValue==TYPE_TWIST)
		{
			geometry_msgs::Twist twist;

			twist.linear.x = bsonView["msg"]["linear"]["x"].get_double();
			twist.linear.y = bsonView["msg"]["linear"]["y"].get_double();
			twist.linear.z = bsonView["msg"]["linear"]["z"].get_double();

			twist.angular.x = bsonView["msg"]["angular"]["x"].get_double();
			twist.angular.y = bsonView["msg"]["angular"]["y"].get_double();
			twist.angular.z = bsonView["msg"]["angular"]["z"].get_double();

			publisherMap[topicValue].publish(twist);
		}
		// CameraInfo
		else if(typeValue==TYPE_CAMERA_INFO)
		{
			sensor_msgs::CameraInfo cameraInfo;

			cameraInfo.header.seq        = (uint32_t)bsonView["msg"]["header"]["seq"]           .get_int32();
			cameraInfo.header.stamp.sec  = (uint32_t)bsonView["msg"]["header"]["stamp"]["secs"] .get_int32();
			cameraInfo.header.stamp.nsec = (uint32_t)bsonView["msg"]["header"]["stamp"]["nsecs"].get_int32();
			cameraInfo.header.frame_id   =           bsonView["msg"]["header"]["frame_id"]      .get_utf8().value.to_string();

			cameraInfo.height            = (uint32_t)bsonView["msg"]["height"].get_int32();
			cameraInfo.width             = (uint32_t)bsonView["msg"]["width"] .get_int32();
			cameraInfo.distortion_model  =           bsonView["msg"]["distortion_model"].get_utf8().value.to_string();

			bsoncxx::array::view dView = bsonView["msg"]["D"].get_array().value;
			cameraInfo.D.resize((size_t)std::distance(dView.cbegin(), dView.cend()));
			setVectorDouble(cameraInfo.D, dView);

			setArrayDouble(cameraInfo.K, bsonView["msg"]["K"].get_array().value);
			setArrayDouble(cameraInfo.R, bsonView["msg"]["R"].get_array().value);
			setArrayDouble(cameraInfo.P, bsonView["msg"]["P"].get_array().value);

			cameraInfo.binning_x         = (uint32_t)bsonView["msg"]["binning_x"].get_int32();
			cameraInfo.binning_y         = (uint32_t)bsonView["msg"]["binning_y"].get_int32();
			cameraInfo.roi.x_offset      = (uint32_t)bsonView["msg"]["roi"]["x_offset"]  .get_int32();
			cameraInfo.roi.y_offset      = (uint32_t)bsonView["msg"]["roi"]["y_offset"]  .get_int32();
			cameraInfo.roi.height        = (uint32_t)bsonView["msg"]["roi"]["height"]    .get_int32();
			cameraInfo.roi.width         = (uint32_t)bsonView["msg"]["roi"]["width"]     .get_int32();
			cameraInfo.roi.do_rectify    = (uint8_t) bsonView["msg"]["roi"]["do_rectify"].get_bool();

			publisherMap[topicValue].publish(cameraInfo);
		}
		// Image
		else if(typeValue==TYPE_IMAGE)
		{
			sensor_msgs::Image image;

			image.header.seq        = (uint32_t)bsonView["msg"]["header"]["seq"]           .get_int32();
			image.header.stamp.sec  = (uint32_t)bsonView["msg"]["header"]["stamp"]["secs"] .get_int32();
			image.header.stamp.nsec = (uint32_t)bsonView["msg"]["header"]["stamp"]["nsecs"].get_int32();
			image.header.frame_id   =           bsonView["msg"]["header"]["frame_id"]      .get_utf8().value.to_string();
			image.height            = (uint32_t)bsonView["msg"]["height"]      .get_int32();
			image.width             = (uint32_t)bsonView["msg"]["width"]       .get_int32();
			image.encoding          =           bsonView["msg"]["encoding"]    .get_utf8().value.to_string();
			image.is_bigendian      =           bsonView["msg"]["is_bigendian"].raw()[0];
			image.step              = (uint32_t)bsonView["msg"]["step"]        .get_int32();

			size_t sizet = (image.step * image.height);
			image.data.resize(sizet);
			memcpy(&image.data[0], bsonView["msg"]["data"].get_binary().bytes, sizet);

			publisherMap[topicValue].publish(image);
		}
		// LaserScan
		else if(typeValue==TYPE_LASER_SCAN)
		{
			sensor_msgs::LaserScan laserScan;

			laserScan.header.seq        = (uint32_t)bsonView["msg"]["header"]["seq"]           .get_int32();
			laserScan.header.stamp.sec  = (uint32_t)bsonView["msg"]["header"]["stamp"]["secs"] .get_int32();
			laserScan.header.stamp.nsec = (uint32_t)bsonView["msg"]["header"]["stamp"]["nsecs"].get_int32();
			laserScan.header.frame_id   =           bsonView["msg"]["header"]["frame_id"]      .get_utf8().value.to_string();

			laserScan.angle_min       = (float)bsonView["msg"]["angle_min"]      .get_double();
			laserScan.angle_max       = (float)bsonView["msg"]["angle_max"]      .get_double();
			laserScan.angle_increment = (float)bsonView["msg"]["angle_increment"].get_double();
			laserScan.time_increment  = (float)bsonView["msg"]["time_increment"] .get_double();
			laserScan.scan_time       = (float)bsonView["msg"]["scan_time"]      .get_double();
			laserScan.range_min       = (float)bsonView["msg"]["range_min"]      .get_double();
			laserScan.range_max       = (float)bsonView["msg"]["range_max"]      .get_double();

			size_t sizet = (size_t)((laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment + 1);

			laserScan.ranges.resize(sizet);
			laserScan.intensities.resize(sizet);

			bsoncxx::array::view dView_ranges = bsonView["msg"]["ranges"].get_array().value;
			laserScan.ranges.resize(std::distance(dView_ranges.cbegin(), dView_ranges.cend()));
			setVectorFloat(laserScan.ranges, dView_ranges);

			bsoncxx::array::view dView_intensities = bsonView["msg"]["intensities"].get_array().value;
			laserScan.intensities.resize(std::distance(dView_intensities.cbegin(), dView_intensities.cend()));
			setVectorFloat(laserScan.intensities, dView_intensities);

			publisherMap[topicValue].publish(laserScan);
		}
		// Tf list data (sigverse original type)
		else if(typeValue==TYPE_TF_LIST)
		{
			static tf::TransformBroadcaster transformBroadcaster;

			bsoncxx::array::view tfArrayView = bsonView["msg"].get_array().value;

			std::vector<tf::StampedTransform> stampedTransformList;

			int i = 0;

			for(auto itr = tfArrayView.cbegin(); itr != tfArrayView.cend(); ++itr)
			{
				std::string frameId      = (*itr)["header"]["frame_id"].get_utf8().value.to_string();
				std::string childFrameId = (*itr)["child_frame_id"]    .get_utf8().value.to_string();

				tf::Vector3 position = tf::Vector3
				(
					(double)(*itr)["transform"]["translation"]["x"].get_double(),
					(double)(*itr)["transform"]["translation"]["y"].get_double(),
					(double)(*itr)["transform"]["translation"]["z"].get_double()
				);

				tf::Quaternion quaternion = tf::Quaternion
				(
					(double)(*itr)["transform"]["rotation"]["x"].get_double(),
					(double)(*itr)["transform"]["rotation"]["y"].get_double(),
					(double)(*itr)["transform"]["rotation"]["z"].get_double(),
					(double)(*itr)["transform"]["rotation"]["w"].get_double()
				);

				tf::Transform transform;
				transform.setOrigin(position);
				transform.setRotation(quaternion);

				stampedTransformList.push_back(tf::StampedTransform(transform, ros::Time::now(), frameId, childFrameId));
			}

			transformBroadcaster.sendTransform(stampedTransformList);
		}

//		std::cout << "published. topic=" << topicValue << std::endl;

		ros::spinOnce();

//		loop_rate.sleep();
	}

	return NULL;
}


int SIGVerseROSBridge::run(int argc, char **argv)
{
	signal(SIGINT, interruptEventHandler);

	uint16_t portNumber;

	// Set port number
	if(argc > 1)
	{
		portNumber = (uint16_t)std::atoi(argv[1]);
	}
	else
	{
		portNumber = PORT;
	}

	int srcSocket;
	struct sockaddr_in srcAddr;

	// TCP connection setting
	bzero((char *)&srcAddr, sizeof(srcAddr));
	srcAddr.sin_port = htons(portNumber);
	srcAddr.sin_family = AF_INET;
	srcAddr.sin_addr.s_addr = INADDR_ANY;

	srcSocket = socket(AF_INET, SOCK_STREAM, 0);

	bind(srcSocket, (struct sockaddr *)&srcAddr, sizeof(srcAddr));

	listen(srcSocket, 100);

	std::cout << "Waiting for connection... port=" << portNumber << std::endl;

	while(true)
	{
		int dstSocket;
		struct sockaddr_in dstAddr;
		int dstAddrSize = sizeof(dstAddr);

		if(!checkReceivable(srcSocket))
		{
			if(isInterrupted){ break; }
			continue;
		}

		dstSocket = accept(srcSocket, (struct sockaddr *)&dstAddr, (socklen_t *)&dstAddrSize);

		std::cout << "Connected from IP=" << inet_ntoa(dstAddr.sin_addr) << " Port=" << dstAddr.sin_port << std::endl;

		pthread_t thread;
		pthread_create( &thread, NULL, SIGVerseROSBridge::receivingThread, (void *)(&dstSocket) );

		pthread_detach(thread);
	}

	close(srcSocket);

	return 0;
}


int main(int argc, char **argv)
{
	SIGVerseROSBridge sigverseROSBridge;
	sigverseROSBridge.run(argc, argv);
};

