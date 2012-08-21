#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>

ros::Publisher transform_pub;

void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  std::cout << "cloud callback..." << std::endl;
  sensor_msgs::PointCloud2 cloud_out;
/*
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  transform.setRotation( tf::Quaternion(0, 1, 0) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "test_frame"));
*/


  tf::StampedTransform transform;
  static tf::TransformListener tf_listener;

try {
  tf_listener.lookupTransform("/test_frame", cloud->header.frame_id, cloud->header.stamp, transform);
  pcl_ros::transformPointCloud("/test_frame", *cloud, cloud_out, tf_listener);
  transform_pub.publish(cloud_out);

} catch (tf::TransformException ex){
  ROS_ERROR("%s", ex.what());
}
///  cloud_out.header.frame_id = "/test_frame";

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_transformPointCloud");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("camera/depth/points", 1, cloudCb);
  transform_pub = nh.advertise<sensor_msgs::PointCloud2>("transform_Pointcloud", 1);

  
ros::spin();

  return 0;
}
