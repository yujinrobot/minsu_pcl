#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  sensor_msgs::PointCloud2 cloud_filtered;
  std::cout << "PointCloud before filtering : " << cloud->width * cloud->height << std::endl;
  // Create the filtering object
  pcl::PassThrough<sensor_msgs::PointCloud2> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0,1.0); // unit : meter
  pass.filter(cloud_filtered);
  std::cout << "PointCloud after filtering : " << cloud_filtered.width * cloud_filtered.height << std::endl;

  // Publish the data
  pub.publish (cloud_filtered);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "passthrough");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("cloud", 1, cloudCb);
  pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered",1);

  ros::spin();
}
