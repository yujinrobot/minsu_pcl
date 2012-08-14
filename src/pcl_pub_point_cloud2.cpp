#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);

  sensor_msgs::PointCloud2 cloud_msg;

  cloud_msg.header.frame_id = "some_tf_frame";
  cloud_msg.height = cloud_msg.width = 1;

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    cloud_msg.header.stamp = ros::Time::now ();
    pub.publish (cloud_msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
