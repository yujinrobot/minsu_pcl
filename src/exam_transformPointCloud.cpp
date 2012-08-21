#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>

#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
//#include <transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>

ros::Publisher transform_pub;

void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  //sensor_msgs::PointCloud2::Ptr cloud_out (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2 cloud_out;
  tf::TransformListener tf_listener;

  pcl_ros::transformPointCloud("/camera_link", *cloud, cloud_out, tf_listener);

  transform_pub.publish(cloud_out);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_transformPointCloud");
  ros::NodeHandle nh;

  transform_pub = nh.advertise<sensor_msgs::PointCloud2>("transform_Pointcloud", 1);

  ros::spin();

  return 0;

}
