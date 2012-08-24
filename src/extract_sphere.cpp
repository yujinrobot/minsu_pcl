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

#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *  This source code used lower sequence
 *
 *  cloud -> passthrough filter -> estimate normal -> sphere
 *
 *  handling of pass through filter
 *  we set a boundary value about floor.
 *
 *  subscribe topic : /transformed_frame_Pointcloud
 *  This topic is transformed by transform_frame_PointCloud.cpp
 *
 *
 *  Added visualize normals
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
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  // Normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentation_from_normals;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZ> ());

  // The plane and sphere coefficients
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients ());
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients ());
  pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients ());

  // The plane and sphere inliers
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices ());
  pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices ());

  // The point clouds
  sensor_msgs::PointCloud2::Ptr voxelgrid_filtered (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr passthrough_filtered (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr plane_output_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr rest_output_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr rest_cloud_filtered (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr cylinder_output_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr sphere_output_cloud (new sensor_msgs::PointCloud2);


  // The PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr remove_transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_output (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_output (new pcl::PointCloud<pcl::PointXYZ>);

  // The cloud normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());        // for plane
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal> ());       // for cylinder
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals3 (new pcl::PointCloud<pcl::Normal> ());       // for sphere


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
  pass.setFilterLimits (0, 3.5);
  pass.filter (*passthrough_filtered);

  pass.setInputCloud (passthrough_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.1, 0.3);
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


  ros::Time normal_start = ros::Time::now();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*passthrough_filtered, *sphere_cloud);

  // Estimate point normals
  normal_estimation.setSearchMethod (tree3);
  normal_estimation.setInputCloud (sphere_cloud);
  normal_estimation.setKSearch (30);
  normal_estimation.compute (*cloud_normals3);

  ros::Time normal_end = ros::Time::now();


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
   * visualize normals
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(sphere_cloud, cloud_normals3);

  while (!viewer.wasStopped ())
  {
      viewer.spinOnce ();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




  ros::Time sphere_start = ros::Time::now();

  // Create the segmentation object for sphere segmentation and set all the paopennirameters
  segmentation_from_normals.setOptimizeCoefficients (true);
  //segmentation_from_normals.setModelType (pcl::SACMODEL_SPHERE);
  segmentation_from_normals.setModelType (pcl::SACMODEL_SPHERE);
  segmentation_from_normals.setMethodType (pcl::SAC_RANSAC);
  segmentation_from_normals.setNormalDistanceWeight (0.1);
  segmentation_from_normals.setMaxIterations (10000);
  segmentation_from_normals.setDistanceThreshold (0.05);
  segmentation_from_normals.setRadiusLimits (0, 0.15);
  segmentation_from_normals.setInputCloud (sphere_cloud);
  segmentation_from_normals.setInputNormals (cloud_normals3);

  // Obtain the sphere inliers and coefficients
  segmentation_from_normals.segment (*inliers_sphere, *coefficients_sphere);
  //std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;

  // Publish the sphere cloud
  extract_indices.setInputCloud (sphere_cloud);
  extract_indices.setIndices (inliers_sphere);
  extract_indices.setNegative (false);
  extract_indices.filter (*sphere_output);

  if (sphere_output->points.empty ())
     std::cerr << "Can't find the sphere component." << std::endl;

  pcl::toROSMsg (*sphere_output, *sphere_output_cloud);
  sphere_pub.publish(sphere_output_cloud);

  ros::Time sphere_end = ros::Time::now();

  std::cout << "cloud size         : " << cloud->width * cloud->height << std::endl;
  std::cout << "pass_th size       : " << passthrough_filtered->width * passthrough_filtered->height << std::endl;
  std::cout << "sphere size        : " << sphere_output_cloud->width * sphere_output_cloud->height << std::endl;
  std::cout << "sphere normal size : " << cloud_normals3->width * cloud_normals3->height << std::endl;

  ros::Time whole_end = ros::Time::now();

  printf("\n");

  std::cout << "whole time             : " << whole_end - whole_start << " sec" << std::endl;
  std::cout << "declare types time     : " << declare_types_end - declare_types_start << " sec" << std::endl;
  std::cout << "passthrough time       : " << pass_end - pass_start << " sec" << std::endl;
  std::cout << "normal estimate time   : " << normal_end - normal_start << " sec" << std::endl;
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
  //ros::Subscriber sub = nh.subscribe("camera/depth/points", 1, callback);
  ros::Subscriber sub = nh.subscribe("transformed_frame_Pointcloud", 1, callback);
  passthrough_pub = nh.advertise<sensor_msgs::PointCloud2> ("passthrough_cloud", 1);
  sphere_pub = nh.advertise<sensor_msgs::PointCloud2> ("sphere_cloud", 1);

  ros::spin();

  return (0);
}
