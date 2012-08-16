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

ros::Publisher rest_pub;
ros::Publisher plane_pub;
ros::Publisher sphere_pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  // filter
  pcl::VoxelGrid<sensor_msgs::PointCloud2> voxel_grid;
  pcl::PassThrough<sensor_msgs::PointCloud2> pass;
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  // Normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentation_from_normals;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  // The plane and sphere coefficients
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients ());
  pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients ());

  // The plane and sphere inliers  // The point clouds
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());
  pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices ());
  // The point clouds
  //sensor_msgs::PointCloud2::Ptr downsampled (new sensor_msgs::PointCloud2);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr extract_cloud (new pcl::PointCloud<pcl::PointXYZ>);


  sensor_msgs::PointCloud2::Ptr plane_output_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr rest_output_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr sphere_output_cloud (new sensor_msgs::PointCloud2);

  sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr remove_transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere (new pcl::PointCloud<pcl::PointXYZ>);

  // The cloud normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*cloud, *transformed_cloud);

  // Estimate point normals
  normal_estimation.setSearchMethod (tree);
  normal_estimation.setInputCloud (transformed_cloud);
  normal_estimation.setKSearch (50);
  normal_estimation.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  segmentation_from_normals.setOptimizeCoefficients (true);
  segmentation_from_normals.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  segmentation_from_normals.setNormalDistanceWeight (0.1);
  segmentation_from_normals.setMethodType (pcl::SAC_RANSAC);
  segmentation_from_normals.setMaxIterations (100);
  segmentation_from_normals.setDistanceThreshold (0.03);
  segmentation_from_normals.setInputCloud (transformed_cloud);
  segmentation_from_normals.setInputNormals (cloud_normals);

  // Obtain the plane inliers and coefficients
  segmentation_from_normals.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract_indices.setInputCloud (transformed_cloud);
  extract_indices.setIndices (inliers_plane);
  extract_indices.setNegative (false);
  extract_indices.filter (*cloud_plane);

  pcl::toROSMsg (*cloud_plane, *plane_output_cloud);
  plane_pub.publish(plane_output_cloud);

  // Remove the planar inliers, extract the rest
  //extract_indices.setNegative (true);
  //extract_indices.filter (*transformed_cloud);
  //extract_normals.setNegative (true);
  //extract_normals.setInputCloud (cloud_normals);
  //extract_normals.setIndices (inliers_plane);
  //extract_normals.filter (*cloud_normals);

  // Create the filtering object
  extract_indices.setNegative (true);
  extract_indices.filter (*remove_transformed_cloud);
  transformed_cloud.swap (remove_transformed_cloud);

  // publish result of Removal the planar inliers, extract the rest
  pcl::toROSMsg (*transformed_cloud, *rest_output_cloud);
  rest_pub.publish(rest_output_cloud);


  // Create the segmentation object for sphere segmentation and set all the paopennirameters
  segmentation_from_normals.setOptimizeCoefficients (true);
  //segmentation_from_normals.setModelType (pcl::SACMODEL_SPHERE);
  segmentation_from_normals.setModelType (pcl::SACMODEL_CYLINDER);
  segmentation_from_normals.setMethodType (pcl::SAC_RANSAC);
  segmentation_from_normals.setNormalDistanceWeight (0.1);
  segmentation_from_normals.setMaxIterations (10000);
  segmentation_from_normals.setDistanceThreshold (0.05);
  segmentation_from_normals.setRadiusLimits (0, 0.1);
  segmentation_from_normals.setInputCloud (transformed_cloud);
  segmentation_from_normals.setInputNormals (cloud_normals);

  // Obtain the sphere inliers and coefficients
  segmentation_from_normals.segment (*inliers_sphere, *coefficients_sphere);
  std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;

  // Publish the sphere cloud
  extract_indices.setInputCloud (transformed_cloud);
  extract_indices.setIndices (inliers_sphere);
  extract_indices.setNegative (false);
  extract_indices.filter (*cloud_sphere);

  pcl::toROSMsg (*cloud_sphere, *sphere_output_cloud);
  rest_pub.publish(sphere_output_cloud);

}


int
main (int argc, char** argv)
{
 // INITIALIZE ROS
   ros::init (argc, argv, "extract_indices");
   ros::NodeHandle nh;
   ros::Subscriber sub = nh.subscribe("camera/depth/points", 1, callback);
   rest_pub = nh.advertise<sensor_msgs::PointCloud2> ("rest_cloud_filtered", 1);
   plane_pub = nh.advertise<sensor_msgs::PointCloud2> ("plane_cloud_filtered", 1);
   sphere_pub = nh.advertise<sensor_msgs::PointCloud2> ("sphere_cloud_filtered", 1);

   ros::spin();

   return (0);
}
