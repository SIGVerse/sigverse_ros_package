#include "sigverse_ros_bridge.hpp"

bool SIGVerseROSBridge::isRunning;
int  SIGVerseROSBridge::syncTimeCnt;
int  SIGVerseROSBridge::syncTimeMaxNum;

pid_t SIGVerseROSBridge::gettid(void)
{
  return syscall(SYS_gettid);
}

void SIGVerseROSBridge::rosSigintHandler([[maybe_unused]] int sig)
{
  isRunning = false;

  rclcpp::shutdown();
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

template <size_t ArrayNum>
void SIGVerseROSBridge::setArrayDouble(std::array<double, ArrayNum> &destArray, const bsoncxx::array::view &arrayView) {
    int i = 0;
    for (auto itr = arrayView.cbegin(); itr != arrayView.cend(); ++itr) {
        destArray[i++] = (*itr).get_double();
    }
}

template <size_t ArrayNum>
void SIGVerseROSBridge::setArrayDouble(boost::array<double, ArrayNum> &destArray, const bsoncxx::array::view &arrayView) {
    int i = 0;
    for (auto itr = arrayView.cbegin(); itr != arrayView.cend(); ++itr) {
        destArray[i++] = (*itr).get_double();
    }
}

void * SIGVerseROSBridge::receivingThread(void *param)
{
  int dstSocket = *((int *)param);

  char *buf;
  buf = new char [BUFFER_SIZE];

  if(buf == NULL)
  {
    std::cout << "Cannot malloc!" << std::endl;
    exit(EXIT_FAILURE);
  }

  long int totalReceivedSize;

  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> twistPublisherMap;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> cameraInfoPublisherMap;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> imagePublisherMap;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> laserScanPublisherMap;

  std::cout << "Socket open. tid=" << gettid() << std::endl;

  auto node = rclcpp::Node::make_shared("sigverse_ros_bridge_" + std::to_string(gettid()));

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, rosSigintHandler);

  rclcpp::Rate loop_rate(100);

  while(rclcpp::ok())
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

    std::cout << "msg size=" << msgSize << std::endl;

    // Get BSON data
    while(msgSize!=totalReceivedSize)
    {
      size_t unreceivedSize = msgSize - (size_t)totalReceivedSize;

      if(!checkReceivable(dstSocket)){ break; }

      long int receivedSize = read(dstSocket, &(buf[totalReceivedSize]), unreceivedSize);

      totalReceivedSize += receivedSize;
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

    std::string opValue    = bsonView["op"]   .get_utf8().value.to_string();
    std::string topicValue = bsonView["topic"].get_utf8().value.to_string();
    std::string typeValue  = bsonView["type"] .get_utf8().value.to_string();
//    std::cout << "op:" << opValue << std::endl;
//    std::cout << "tp:" << topicValue << std::endl;
//    std::cout << "tv:" << typeValue << std::endl;

    // Advertise
    if(typeValue!=TYPE_TIME_SYNC && typeValue!=TYPE_TF_LIST)
    {
      if(typeValue==TYPE_TWIST)
      {
        if(twistPublisherMap.count(topicValue)==0)
        {
          auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(topicValue, 1000);
          twistPublisherMap[topicValue] = publisher;
        }
      }
      else if(typeValue==TYPE_CAMERA_INFO)
      {
        if(cameraInfoPublisherMap.count(topicValue)==0)
        {
          auto publisher = node->create_publisher<sensor_msgs::msg::CameraInfo>(topicValue, 10);
          cameraInfoPublisherMap[topicValue] = publisher;
        }
      }
      else if(typeValue==TYPE_IMAGE)
      {
        if(imagePublisherMap.count(topicValue)==0)
        {
          auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topicValue, 10);
          imagePublisherMap[topicValue] = publisher;
        }
      }
      else if(typeValue==TYPE_LASER_SCAN)
      {
        if(laserScanPublisherMap.count(topicValue)==0)
        {
          auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(topicValue, 10);
          laserScanPublisherMap[topicValue] = publisher;
        }
      }
      else
      {
        std::cout << "Not compatible message type! :" << typeValue << std::endl;
        continue;
      }

      std::cout << "Advertised " << topicValue << std::endl;
    }

    // Publish
    // Twist
    if(typeValue==TYPE_TWIST)
    {
      geometry_msgs::msg::Twist twist;

      twist.linear.x = bsonView["msg"]["linear"]["x"].get_double();
      twist.linear.y = bsonView["msg"]["linear"]["y"].get_double();
      twist.linear.z = bsonView["msg"]["linear"]["z"].get_double();

      twist.angular.x = bsonView["msg"]["angular"]["x"].get_double();
      twist.angular.y = bsonView["msg"]["angular"]["y"].get_double();
      twist.angular.z = bsonView["msg"]["angular"]["z"].get_double();

      twistPublisherMap[topicValue]->publish(twist);
    }
    // CameraInfo
    else if(typeValue==TYPE_CAMERA_INFO)
    {
      sensor_msgs::msg::CameraInfo cameraInfo;

      int32_t sec  = bsonView["msg"]["header"]["stamp"]["sec"]    .get_int32();
      int32_t nsec = bsonView["msg"]["header"]["stamp"]["nanosec"].get_int32(); // get_uint32() is not available; use get_int32() and cast if safe.
      cameraInfo.header.stamp = rclcpp::Time(static_cast<uint64_t>(sec) * 1000000000ULL + static_cast<uint64_t>(nsec));
      cameraInfo.header.frame_id   =           bsonView["msg"]["header"]["frame_id"]      .get_utf8().value.to_string();

      cameraInfo.height            = (uint32_t)bsonView["msg"]["height"].get_int32();
      cameraInfo.width             = (uint32_t)bsonView["msg"]["width"] .get_int32();
      cameraInfo.distortion_model  =           bsonView["msg"]["distortion_model"].get_utf8().value.to_string();

      bsoncxx::array::view dView = bsonView["msg"]["d"].get_array().value;
      cameraInfo.d.resize((size_t)std::distance(dView.cbegin(), dView.cend()));
      setVectorDouble(cameraInfo.d, dView);

      setArrayDouble(cameraInfo.k, bsonView["msg"]["k"].get_array().value);
      setArrayDouble(cameraInfo.r, bsonView["msg"]["r"].get_array().value);
      setArrayDouble(cameraInfo.p, bsonView["msg"]["p"].get_array().value);

      cameraInfo.binning_x         = (uint32_t)bsonView["msg"]["binning_x"].get_int32();
      cameraInfo.binning_y         = (uint32_t)bsonView["msg"]["binning_y"].get_int32();
      cameraInfo.roi.x_offset      = (uint32_t)bsonView["msg"]["roi"]["x_offset"]  .get_int32();
      cameraInfo.roi.y_offset      = (uint32_t)bsonView["msg"]["roi"]["y_offset"]  .get_int32();
      cameraInfo.roi.height        = (uint32_t)bsonView["msg"]["roi"]["height"]    .get_int32();
      cameraInfo.roi.width         = (uint32_t)bsonView["msg"]["roi"]["width"]     .get_int32();
      cameraInfo.roi.do_rectify    = (uint8_t) bsonView["msg"]["roi"]["do_rectify"].get_bool();

      cameraInfoPublisherMap[topicValue]->publish(cameraInfo);
    }
    // Image
    else if(typeValue==TYPE_IMAGE)
    {
      sensor_msgs::msg::Image image;

      int32_t sec  = bsonView["msg"]["header"]["stamp"]["sec"]    .get_int32();
      int32_t nsec = bsonView["msg"]["header"]["stamp"]["nanosec"].get_int32(); // get_uint32() is not available; use get_int32() and cast if safe.
      image.header.stamp = rclcpp::Time(static_cast<uint64_t>(sec) * 1000000000ULL + static_cast<uint64_t>(nsec));
      image.header.frame_id   =           bsonView["msg"]["header"]["frame_id"]      .get_utf8().value.to_string();
      image.height            = (uint32_t)bsonView["msg"]["height"]      .get_int32();
      image.width             = (uint32_t)bsonView["msg"]["width"]       .get_int32();
      image.encoding          =           bsonView["msg"]["encoding"]    .get_utf8().value.to_string();
      image.is_bigendian      = (uint8_t) bsonView["msg"]["is_bigendian"].get_int32(); //.raw()[0];
      image.step              = (uint32_t)bsonView["msg"]["step"]        .get_int32();

      size_t sizet = (image.step * image.height);
      image.data.resize(sizet);
      memcpy(&image.data[0], bsonView["msg"]["data"].get_binary().bytes, sizet);

      imagePublisherMap[topicValue]->publish(image);
    }
    // LaserScan
    else if(typeValue==TYPE_LASER_SCAN)
    {
      sensor_msgs::msg::LaserScan laserScan;

      int32_t sec  = bsonView["msg"]["header"]["stamp"]["sec"]    .get_int32();
      int32_t nsec = bsonView["msg"]["header"]["stamp"]["nanosec"].get_int32(); // get_uint32() is not available; use get_int32() and cast if safe.
      laserScan.header.stamp = rclcpp::Time(static_cast<uint64_t>(sec) * 1000000000ULL + static_cast<uint64_t>(nsec));
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

      laserScanPublisherMap[topicValue]->publish(laserScan);
    }
    // Time Synchronization (SIGVerse Original Type)
    else if(typeValue==TYPE_TIME_SYNC)
    {
      if(syncTimeCnt < syncTimeMaxNum)
      {
        int32_t sec  = bsonView["msg"]["sec"]    .get_int32();
        int32_t nsec = bsonView["msg"]["nanosec"].get_int32(); // get_uint32() is not available; use get_int32() and cast if safe.

        rclcpp::Clock clock;
		    double now_sec = clock.now().seconds();

        int gapSec  = sec  - static_cast<int>(now_sec);
        int gapMsec = static_cast<int>((nsec / 1e6) - ((now_sec - static_cast<int>(now_sec)) * 1e3));

        std::string timeGap = "time_gap," + std::to_string(gapSec) + "," + std::to_string(gapMsec);

//        ssize_t size = 
		    write(dstSocket, timeGap.c_str(), std::strlen(timeGap.c_str()));

        std::cout << "TYPE_TIME_SYNC " << timeGap.c_str() << std::endl;

        syncTimeCnt++;
      }
    }
    // Tf list data (SIGVerse Original Type)
    else if(typeValue==TYPE_TF_LIST)
    {
      static std::shared_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster;
      if (!transformBroadcaster)
        transformBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

      bsoncxx::array::view tfArrayView = bsonView["msg"].get_array().value;

      std::vector<geometry_msgs::msg::TransformStamped> stampedTransformList;

      for(auto itr = tfArrayView.cbegin(); itr != tfArrayView.cend(); ++itr)
      {
        std::string frameId      = (*itr)["header"]["frame_id"].get_utf8().value.to_string();
        int32_t sec              = (*itr)["header"]["stamp"]["sec"]    .get_int32();
        int32_t nsec             = (*itr)["header"]["stamp"]["nanosec"].get_int32(); // get_uint32() is not available; use get_int32() and cast if safe.
        std::string childFrameId = (*itr)["child_frame_id"]    .get_utf8().value.to_string();

        geometry_msgs::msg::TransformStamped stampedTransform;

        if (sec == 0) {
          stampedTransform.header.stamp = node->get_clock()->now();
        } else {
          stampedTransform.header.stamp = rclcpp::Time(static_cast<uint64_t>(sec) * 1000000000ULL + static_cast<uint64_t>(nsec));
        }
        stampedTransform.header.frame_id = frameId;
        stampedTransform.child_frame_id = childFrameId;

        stampedTransform.transform.translation.x = (*itr)["transform"]["translation"]["x"].get_double();
        stampedTransform.transform.translation.y = (*itr)["transform"]["translation"]["y"].get_double();
        stampedTransform.transform.translation.z = (*itr)["transform"]["translation"]["z"].get_double();

        stampedTransform.transform.rotation.x = (*itr)["transform"]["rotation"]["x"].get_double();
        stampedTransform.transform.rotation.y = (*itr)["transform"]["rotation"]["y"].get_double();
        stampedTransform.transform.rotation.z = (*itr)["transform"]["rotation"]["z"].get_double();
        stampedTransform.transform.rotation.w = (*itr)["transform"]["rotation"]["w"].get_double();

        stampedTransformList.push_back(stampedTransform);
      }

      transformBroadcaster->sendTransform(stampedTransformList);
    }

//    std::cout << "published. topic=" << topicValue << std::endl;

    rclcpp::spin_some(node);
  }

  delete[] buf;

  return NULL;
}


int SIGVerseROSBridge::run(int argc, char **argv)
{
  uint16_t portNumber;

  portNumber = DEFAULT_PORT;
  syncTimeMaxNum = DEFAULT_SYNC_TIME_MAX_NUM;

  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if (arg == "--ros-args") { break; }

    if(i==1)
    {
      portNumber = (uint16_t)std::atoi(argv[1]);
      std::cout << "argv[1] portNumber=" << argv[1] << std::endl;
    }
    if(i==2)
    {
      syncTimeMaxNum = (uint16_t)std::atoi(argv[2]);
      std::cout << "argv[2] syncTimeMaxNum=" << argv[2] << std::endl;
    }
  }

  isRunning = true;
  syncTimeCnt = 0;

  int srcSocket;
  struct sockaddr_in srcAddr;

  // TCP connection setting
//  bzero((char *)&srcAddr, sizeof(srcAddr));
  memset(&srcAddr, 0, sizeof(srcAddr));
  srcAddr.sin_port = htons(portNumber);
  srcAddr.sin_family = AF_INET;
  srcAddr.sin_addr.s_addr = INADDR_ANY;

  srcSocket = socket(AF_INET, SOCK_STREAM, 0);

  bind(srcSocket, (struct sockaddr *)&srcAddr, sizeof(srcAddr));

  listen(srcSocket, 100);

  std::cout << "Waiting for connection... port=" << portNumber << std::endl;

  while(isRunning)
  {
    int dstSocket;

    struct sockaddr_in dstAddr;
    int dstAddrSize = sizeof(dstAddr);

    if(!checkReceivable(srcSocket))
    {
      continue;
    }

    dstSocket = accept(srcSocket, (struct sockaddr *)&dstAddr, (socklen_t *)&dstAddrSize);

    std::cout << "Connected from IP=" << inet_ntoa(dstAddr.sin_addr) << " Port=" << dstAddr.sin_port << std::endl;

    pthread_t thread;
    pthread_create( &thread, NULL, receivingThread, (void *)(&dstSocket));
    pthread_detach(thread);

    usleep(10 * 1000);
  }

  close(srcSocket);

  return 0;
}


int main(int argc, char **argv)
{
  std::cout << "pid=" << getpid() << std::endl;

  rclcpp::init(argc, argv);

  SIGVerseROSBridge sigverseROSBridge;
  sigverseROSBridge.run(argc, argv);

  rclcpp::shutdown();
};

