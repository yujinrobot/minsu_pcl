#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/foreach.hpp>

void cloudCb(const sensor_msgs::PointCloud2::Ptr& cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud, *point_cloud);

  if(point_cloud->points.empty()) {
    std::cerr << "Can't subscribe the sphere point cloud information." << std::endl;
  }

  printf ("Cloud: width = %d, height = %d size : %d\n", point_cloud->width, point_cloud->height, point_cloud->width*point_cloud->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, point_cloud->points)
  {
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracking_ball_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub_cloud = nh.subscribe("sphere_cloud", 1, cloudCb);
  return 0;

}
