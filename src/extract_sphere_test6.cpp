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

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *  This source code used lower sequence
 *
 *  cloud -> passthrough filter -> sphere
 *
 *  handling of pass through filter
 *  we set a boundary value about floor.
 *
 *  subscribe topic : /camera/depth/points
 *
 *  1st - segmentation
 *  2nd - RANSAC
 *
 *
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ros::Publisher passthrough_pub;
ros::Publisher sphere_seg_pub;
ros::Publisher sphere_RANSAC_pub;

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

  std::vector<int> inliers;

  // The plane and sphere coefficients
  pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients ());

  // The plane and sphere inliers
  pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices ());

  // The point clouds
  sensor_msgs::PointCloud2::Ptr voxelgrid_filtered (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr passthrough_filtered (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr sphere_seg_output_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr sphere_RANSAC_output_cloud (new sensor_msgs::PointCloud2);


  // The PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_seg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_seg_output (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_RANSAC_output (new pcl::PointCloud<pcl::PointXYZ>);


  ros::Time declare_types_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
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
  pass.setFilterLimits (-0.1, 0.27);
  pass.filter (*passthrough_filtered);

  passthrough_pub.publish(passthrough_filtered);

  ros::Time pass_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
   * for sphere features pcl::SACSegmentation
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ros::Time sphere_seg_start = ros::Time::now();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*passthrough_filtered, *sphere_seg_cloud);

  // Optional
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_SPHERE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.15);
  seg.setInputCloud (sphere_seg_cloud);
  seg.segment (*inliers_sphere, *coefficients_sphere);

  extract_indices.setInputCloud(sphere_seg_cloud);
  extract_indices.setIndices(inliers_sphere);
  extract_indices.setNegative(false);
  extract_indices.filter(*sphere_seg_output);

//  if (sphere_output->points.empty ())sphere_output
//     std::cerr << "Can't find the sphere component." << std::endl;
//
  pcl::toROSMsg (*sphere_seg_output, *sphere_seg_output_cloud);
  sphere_seg_pub.publish(sphere_seg_output_cloud);

  ros::Time sphere_seg_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
   * for sphere features pcl::SampleConsensusModelSphere
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  ros::Time sphere_RANSAC_start = ros::Time::now();

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (sphere_seg_output));

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
  ransac.setDistanceThreshold (.01);
  ransac.computeModel();
  ransac.getInliers(inliers);

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud<pcl::PointXYZ>(*sphere_seg_output, inliers, *sphere_RANSAC_output);

  pcl::toROSMsg (*sphere_RANSAC_output, *sphere_RANSAC_output_cloud);
  sphere_RANSAC_pub.publish(sphere_RANSAC_output_cloud);

//  // Optional
//  seg.setOptimizeCoefficients (true);
//  // Mandatory
//  seg.setModelType (pcl::SACMODEL_SPHERE);
//  seg.setMethodType (pcl::SAC_RANSAC);
//  seg.setMaxIterations (10000);
//  seg.setDistanceThreshold (0.05);
//  seg.setRadiusLimits (0, 0.15);
//  seg.setInputCloud (sphere_cloud);
//  seg.segment (*inliers_sphere, *coefficients_sphere);
//  //std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;
//
//
//  extract_indices.setInputCloud(sphere_cloud);
//  extract_indices.setIndices(inliers_sphere);
//  extract_indices.setNegative(false);
//  extract_indices.filter(*sphere_output);
//
//  if (sphere_output->points.empty ())
//     std::cerr << "Can't find the sphere component." << std::endl;
//
//  pcl::toROSMsg (*sphere_output, *sphere_output_cloud);
//  sphere_pub.publish(sphere_output_cloud);

  ros::Time sphere_RANSAC_end = ros::Time::now();

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

  std::cout << "cloud size                : " << cloud->width * cloud->height << std::endl;
  std::cout << "pass_th size              : " << passthrough_filtered->width * passthrough_filtered->height << std::endl;
  std::cout << "sphere seg seg            : " << sphere_seg_output_cloud->width * sphere_seg_output_cloud->height << std::endl;
  std::cout << "sphere seg seg2           : " << sphere_seg_output->points.size() << std::endl;
  std::cout << "sphere RANSAC size        : " << sphere_RANSAC_output_cloud->width * sphere_RANSAC_output_cloud->height << std::endl;

  printf("\n");

  std::cout << "whole time                    : " << whole_end - whole_start << " sec" << std::endl;
  std::cout << "declare types time            : " << declare_types_end - declare_types_start << " sec" << std::endl;
  std::cout << "passthrough time              : " << pass_end - pass_start << " sec" << std::endl;
  std::cout << "sphere seg time               : " << sphere_seg_end - sphere_seg_start << " sec" << std::endl;
  std::cout << "sphere ransac time            : " << sphere_RANSAC_end - sphere_RANSAC_start << " sec" << std::endl;

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
  sphere_seg_pub = nh.advertise<sensor_msgs::PointCloud2> ("sphere_seg_cloud", 1);
  sphere_RANSAC_pub = nh.advertise<sensor_msgs::PointCloud2> ("sphere_RANSAC_cloud", 1);

  ros::spin();

  return (0);
}


