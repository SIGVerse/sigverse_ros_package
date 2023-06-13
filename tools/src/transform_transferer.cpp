#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sigverse_transform_transferer");
  
  std::string map_frame;
  std::string base_frame;
  
  ros::NodeHandle node;

  node.param<std::string>("sigverse_transform_transferer/map_frame",  map_frame,  "/map");
  node.param<std::string>("sigverse_transform_transferer/base_frame", base_frame, "/base_link");
  
  ros::Publisher pub_transform = node.advertise<geometry_msgs::TransformStamped>("/base_transform", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  
  ros::Time now = ros::Time::now();
  listener.waitForTransform(map_frame, base_frame, now, ros::Duration(30.0));

  while (node.ok())
  {
    tf::StampedTransform transform;

    try
    {
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.frame_id = map_frame;
    transformStamped.child_frame_id = base_frame;

    geometry_msgs::Vector3 pos;
    pos.x = transform.getOrigin().x();
    pos.y = transform.getOrigin().y();
    pos.z = transform.getOrigin().z();
  
    geometry_msgs::Quaternion qua;
    qua.x = transform.getRotation().x();
    qua.y = transform.getRotation().y();
    qua.z = transform.getRotation().z();
    qua.w = transform.getRotation().w();

    transformStamped.transform.translation = pos;
    transformStamped.transform.rotation    = qua;
    
    pub_transform.publish(transformStamped);

    rate.sleep();
  }
  
  return 0;
};
