#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *  This source code used lower sequence
 *
 *  cloud ->  -> plane
 *
 *  handling of pass through filter
 *  we set a boundary value about floor.
 *
 *  subscribe topic : /transformed_frame_Pointcloud
 *  This topic is transformed by transform_frame_PointCloud.cpp
 *
 *  pcl::SACSegmentation
 *
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ros::Publisher passthrough_pub;
ros::Publisher plane_pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
  ros::Time whole_start = ros::Time::now();

  ros::Time declare_types_start = ros::Time::now();

  // filter
  pcl::VoxelGrid<sensor_msgs::PointCloud2> voxel_grid;
  pcl::PassThrough<sensor_msgs::PointCloud2> pass;
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // The plane and sphere coefficients
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients ());

  // The plane and sphere inliers
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());

  // The point clouds
  sensor_msgs::PointCloud2::Ptr plane_seg_output_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr sphere_RANSAC_output_cloud (new sensor_msgs::PointCloud2);

  // The PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_output (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_RANSAC_output (new pcl::PointCloud<pcl::PointXYZ>);


  ros::Time declare_types_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
   * for plane features pcl::SACSegmentation
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ros::Time plane_seg_start = ros::Time::now();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*cloud, *plane_seg_cloud);

  // Optional
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.15);
  seg.setInputCloud (plane_seg_cloud);
  seg.segment (*inliers_plane, *coefficients_plane);

  extract_indices.setInputCloud(plane_seg_cloud);
  extract_indices.setIndices(inliers_plane);
  extract_indices.setNegative(false);
  extract_indices.filter(*plane_seg_output);

//  if (sphere_output->points.empty ())sphere_output
//     std::cerr << "Can't find the sphere component." << std::endl;
//
  pcl::toROSMsg (*plane_seg_output, *plane_seg_output_cloud);
  plane_pub.publish(plane_seg_output_cloud);

  ros::Time plane_seg_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  ros::Time whole_end = ros::Time::now();

  std::cout << "cloud size         : " << cloud->width * cloud->height << std::endl;
  std::cout << "plane size         : " << plane_seg_output_cloud->width * plane_seg_output_cloud->height << std::endl;
  //std::cout << "inliers size       : " << inliers_sphere->indices.size() << std::endl;

  printf("\n");

  std::cout << "whole time             : " << whole_end - whole_start << " sec" << std::endl;
  std::cout << "declare types time     : " << declare_types_end - declare_types_start << " sec" << std::endl;
  std::cout << "plane time             : " << plane_seg_end - plane_seg_start << " sec" << std::endl;

  printf("\n----------------------------------------------------------------------------\n");
  printf("\n");
}


int
main (int argc, char** argv)
{
  // INITIALIZE ROS
  ros::init (argc, argv, "extract_sphere");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("camera/depth/points", 1, callback);
  //ros::Subscriber sub = nh.subscribe("transformed_frame_Pointcloud", 1, callback);

  plane_pub = nh.advertise<sensor_msgs::PointCloud2> ("plane_cloud", 1);

  ros::spin();

  return (0);
}
