#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/foreach.hpp>

ros::Publisher pub_cmd;

void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  //X,Y,Z of the centroid
  //double x = 0.0;
  //double y = 0.0;
  double z = 0.0;

  //Number of points observed
  unsigned int n = 0;
  unsigned int k = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud, *point_cloud);

  if(point_cloud->points.empty()) {
    std::cerr << "Can't subscribe the sphere point cloud information." << std::endl;
  }

//  printf ("Cloud: width = %d, height = %d size : %d\n", point_cloud->width, point_cloud->height, point_cloud->width*point_cloud->height);
//  BOOST_FOREACH (const pcl::PointXYZ& pt, point_cloud->points)
//  {
//    //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
//
//    if(!std::isnan(z)) {
//      z += pt.z;
//      n++;
//    }
//
//    if(n) {
//      z /= n;
//
//      printf ("\t(z : %f) (n : %d)\n", z, n);
//      geometry_msgs::Twist cmd;
//      if(z < 2.0) {
//        cmd.linear.x = 0.2;
//      } else if (z < 1.25) {
//        pub_cmd.publish(geometry_msgs::Twist());
//      }
//      pub_cmd.publish(cmd);
//    }
//    geometry_msgs::Twist cmd;
//
//    if(pt.z < 2.0) {
//      cmd.linear.x = 0.2;
//    } else if (pt.z < 1.25) {
//      pub_cmd.publish(geometry_msgs::Twist());
//    }
//    //pub_cmd.publish(cmd);
//    printf ("\t(z : %f)\n", pt.z);
//
//  }
  BOOST_FOREACH (const pcl::PointXYZ& pt, point_cloud->points)
  {
    //ROS_INFO("%f", pt.z);

    if(!std::isnan(z)) {
      z += pt.z;
      n++;
    }
    k++;

    if(n) {
      printf("z : %f n : %d k : %d\n", z, n, k);

      z /= n;
      geometry_msgs::Twist cmd;

      if(pt.z < 2.0) {
        cmd.linear.x = 0.2;
      } else if (pt.z < 0.8) {
        pub_cmd.publish(geometry_msgs::Twist());
      }
      pub_cmd.publish(cmd);
    }
  }
  printf("\n----------------------------------------------------------------------------\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracking_ball_pcl");
  ros::NodeHandle nh;
  pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub_cloud = nh.subscribe("sphere_cloud", 1, cloudCb);

  ros::spin();

  return 0;

}
