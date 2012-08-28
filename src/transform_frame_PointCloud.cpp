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
  ROS_INFO("PointCloud callback...");

  sensor_msgs::PointCloud2 cloud_out;


//  tf::TransformBroadcaster br;
//  tf::Transform transform;
//  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
//  transform.setRotation( tf::Quaternion(0, 1, 0) );a
//  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "test_frame"));

  tf::StampedTransform transform;
  static tf::TransformListener tf_listener;

  try {

    ros::Time start = ros::Time::now();
    tf_listener.lookupTransform("/offset_frame", cloud->header.frame_id, cloud->header.stamp, transform);
    pcl_ros::transformPointCloud("/offset_frame", *cloud, cloud_out, tf_listener);
    transform_pub.publish(cloud_out);
    ros::Time end = ros::Time::now();

    ROS_ERROR("TIME: %.2fms", (end - start).toSec() * 1000);
  } catch (tf::TransformException& ex){

    ROS_ERROR("%s", ex.what());

  }
///  cloud_out.header.frame_id = "/offset_frame";

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_frame_PointCloud");
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe("camera/depth/points", 1, cloudCb);
  transform_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_frame_Pointcloud", 1);

  
  ros::spin();

  return 0;
}
