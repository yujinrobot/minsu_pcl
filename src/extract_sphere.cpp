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

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  // filter
  pcl::VoxelGrid<sensor_msgs::PointCloud2> voxel_grid;
  pcl::PassThrough<sensor_msgs::PointCloud2> pass;
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  pcl::ExtractIndices<pcl::PointXYZ> extract_normals;

  // Normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentation_from_normals;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  // The plane and sphere coefficients
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients ());

  // The plane and sphere inliers
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());


  sensor_msgs::PointCloud2::Ptr downsampled (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr output_cloud (new sensor_msgs::PointCloud2);

  sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr extract_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);

  // The cloud normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*cloud, *transform_cloud);

  // Estimate point normals
  normal_estimation.setSearchMethod (tree);
  normal_estimation.setInputCloud (transform_cloud);
  normal_estimation.setKSearch (50);
  normal_estimation.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  segmentation_from_normals.setOptimizeCoefficients (true);
  segmentation_from_normals.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  segmentation_from_normals.setNormalDistanceWeight (0.1);
  segmentation_from_normals.setMethodType (pcl::SAC_RANSAC);
  segmentation_from_normals.setMaxIterations (100);
  segmentation_from_normals.setDistanceThreshold (0.03);
  segmentation_from_normals.setInputCloud (transform_cloud);
  segmentation_from_normals.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  segmentation_from_normals.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract_indices.setInputCloud (transform_cloud);
  extract_indices.setIndices (inliers_plane);
  extract_indices.setNegative (false);
  extract_indices.filter (*cloud_plane);

  pcl::toROSMsg (*cloud_plane, *output_cloud);
  pub.publish(output_cloud);

}


int
main (int argc, char** argv)
{
 // INITIALIZE ROS
   ros::init (argc, argv, "extract_indices");
   ros::NodeHandle nh;
   ros::Subscriber sub = nh.subscribe("camera/depth/points", 1, callback);
   pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud_filtered", 1);

   ros::spin();

   return (0);
}
