#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/foreach.hpp>

ros::Publisher pub_cmd;

double minDetect = 0.85;

void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  //X,Y,Z of the centroid
  double x = 0.0;
  //double y = 0.0;
  double z = 0.0;

  //Number of points observed
  unsigned int n = 0;

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud, *point_cloud);

  if(point_cloud->points.empty()) {
    std::cerr << "Can't subscribe the sphere point cloud information." << std::endl;
  }

  BOOST_FOREACH (const pcl::PointXYZ& pt, point_cloud->points)
  {
    //ROS_INFO("%f", pt.z);

    if(!std::isnan(x) && !std::isnan(z)) {
      x += pt.x;	
      z += pt.z;
      n++;
      //ROS_INFO("pt.z : %f z : %f n : %d", pt.z, z, n);
    }
  }
  if(n) {
    //printf("z : %f n : %d k : %d\n", z, n, k);
    x /= n;
    z /= n;

    ROS_INFO("[z : %f x : %f n : %d]", z, x, n);

    geometry_msgs::Twist cmd;

    if(z < 3.0 && z > minDetect) {
      cmd.linear.x = 0.2;
      cmd.angular.z = -x*0.5;
      std::cout<<"cmd_vel command" << std::endl;
    } else if (z < minDetect && z > 0.7) {
      pub_cmd.publish(geometry_msgs::Twist());
      std::cout<<"stop"<<std::endl;
    }
    pub_cmd.publish(cmd);
    ROS_INFO("vel_linear : %f vel_angular : %f", cmd.linear.x, cmd.angular.z);
  }

  printf("----------------------------------------------------------------------------\n");
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
