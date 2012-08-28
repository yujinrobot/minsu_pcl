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
 *  cloud -> passthrough filter -> sphere
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
ros::Publisher sphere_pub;

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
  pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients ());

  // The plane and sphere inliers
  pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices ());

  // The point clouds
  sensor_msgs::PointCloud2::Ptr voxelgrid_filtered (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr passthrough_filtered (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr sphere_output_cloud (new sensor_msgs::PointCloud2);


  // The PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_output (new pcl::PointCloud<pcl::PointXYZ>);


  ros::Time declare_types_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
   * Create Voxel grid Filtering
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  voxel_grid.setInputCloud (cloud);
//  voxel_grid.setLeafSize (0.01, 0.01, 0.01);
//  voxel_grid.filter (*voxelgrid_filtered);
//
//  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
//  pcl::fromROSMsg (*voxelgrid_filtered, *transformed_cloud);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
   * Pass through Filtering
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ros::Time pass_start = ros::Time::now();

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 2.5);
  pass.filter (*passthrough_filtered);

  pass.setInputCloud (passthrough_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (0.0, 0.2);
  pass.filter (*passthrough_filtered);

  passthrough_pub.publish(passthrough_filtered);

  ros::Time pass_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
   * for sphere features
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*passthrough_filtered, *sphere_cloud);

  ros::Time sphere_start = ros::Time::now();

  // Optional
  seg.setOptimizeCoefficients (false);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_SPHERE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.03);
  seg.setRadiusLimits (0.12, 0.16);
  seg.setInputCloud (sphere_cloud);
  seg.segment (*inliers_sphere, *coefficients_sphere);
  //std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;


  if (inliers_sphere->indices.empty())
     std::cerr << "Can't find the sphere component." << std::endl;
  else {
    extract_indices.setInputCloud(sphere_cloud);
    extract_indices.setIndices(inliers_sphere);
    extract_indices.setNegative(false);
    extract_indices.filter(*sphere_output);
    pcl::toROSMsg (*sphere_output, *sphere_output_cloud);
    sphere_pub.publish(sphere_output_cloud);
  }

  ros::Time sphere_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
   * visualize normals
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//  viewer.setBackgroundColor (0.0, 0.0, 0.5);
//  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(sphere_cloud, cloud_normals3);
//  viewer.spin();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  ros::Time whole_end = ros::Time::now();

  std::cout << "cloud size         : " << cloud->width * cloud->height << std::endl;
  std::cout << "pass_th size       : " << passthrough_filtered->width * passthrough_filtered->height << std::endl;
  std::cout << "sphere size        : " << sphere_output_cloud->width * sphere_output_cloud->height << std::endl;
  std::cout << "inliers size       : " << inliers_sphere->indices.size() << std::endl;

  printf("\n");

  std::cout << "whole time             : " << whole_end - whole_start << " sec" << std::endl;
  std::cout << "declare types time     : " << declare_types_end - declare_types_start << " sec" << std::endl;
  std::cout << "passthrough time       : " << pass_end - pass_start << " sec" << std::endl;
  std::cout << "sphere time            : " << sphere_end - sphere_start << " sec" << std::endl;

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
  passthrough_pub = nh.advertise<sensor_msgs::PointCloud2> ("passthrough_cloud", 1);
  sphere_pub = nh.advertise<sensor_msgs::PointCloud2> ("sphere_cloud", 1);

  ros::spin();

  return (0);
}
