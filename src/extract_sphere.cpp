#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  sensor_msgs::PointCloud2::Ptr downsampled(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr extract_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Do some downsampling to the point cloud
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*downsampled);

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*downsampled, *transform_cloud);


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_SPHERE);
  seg.setMethodType (pcl::SAC_RANSAC);
  //segmentation_from_normals.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);
  seg.setRadiusLimits(0.001, 0.09);
  seg.setInputCloud (transform_cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  }
  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  extract.setInputCloud(transform_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*extract_cloud);
  std::cerr << "PointCloud representing the planar component: " << extract_cloud->width * extract_cloud->height << " data points." << std::endl;

  // Create the filtering object
  // extract.setNegative (true);
  // extract.filter (*cloud_f);
  // cloud_filtered.swap (cloud_f);

  // Convert the pcl/PointCloud to sensor_msgs/PointCloud2 data
  pcl::toROSMsg (*extract_cloud, *output_cloud);
  pub.publish(output_cloud);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "extract_sphere");

}
