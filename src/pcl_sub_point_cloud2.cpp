#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>


void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  printf ("Cloud: width = %d, height = %d\n", cloud->width, cloud->height);
  //BOOST_FOREACH (const sensor_msgs::PointCloud2& pt, cloud.data.size())
    //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    //printf ("\t(%f)\n", pt.data[0]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("points2", 1, callback);
  ros::spin();
}
