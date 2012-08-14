#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub;

void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  sensor_msgs::PointCloud2 cloud_filtered;
  std::cout << "PointCloud before filtering : " << cloud->width * cloud->height << std::endl;
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (cloud_filtered);
  std::cout << "PointCloud after filtering : " << cloud_filtered.width * cloud_filtered.height << std::endl;

  pub.publish(cloud_filtered);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "outlier_removal");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("cloud", 1, cloudCb);
  pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered",1);

  ros::spin();
}
