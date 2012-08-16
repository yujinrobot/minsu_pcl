#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// A global point cloud for our subscriber callback.
PointCloud::Ptr g_cloud (new PointCloud);

void callback(const PointCloud::ConstPtr& msg)
{
  // Set the global point cloud to the current message cloud.
  g_cloud->points = msg->points;
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pcl_filter");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
  ros::Publisher pub = nh.advertise<PointCloud> ("filtered", 1);

  // The point clouds
  PointCloud::Ptr msg (new PointCloud);
  PointCloud::Ptr cloud_filtered (new PointCloud);
  PointCloud::Ptr cloud_plane (new PointCloud);
  PointCloud::Ptr cloud_sphere (new PointCloud);

  // The cloud normals
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

  // The filters
  pcl::VoxelGrid<PointT> voxel_grid;
  pcl::PassThrough<PointT> pass_through;
  pcl::ExtractIndices<PointT> extract_indices;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  // Normal estimation
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> segmentation_from_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // The plane and sphere coefficients
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients ()), coefficients_sphere (new pcl::ModelCoefficients ());

  // The plane and sphere inliers
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ()), inliers_sphere (new pcl::PointIndices ());

  ros::Rate loop_rate(10);
  while (nh.ok())
    {

      pass_through.setInputCloud (g_cloud);
      pass_through.setFilterFieldName ("z");
      pass_through.setFilterLimits (0, 1.5);
      pass_through.filter (*cloud_filtered);
      ROS_INFO ("Point Cloud size: %zu", g_cloud->points.size ());
      ROS_INFO ("Point Cloud size after pass through: %zu", cloud_filtered->points.size ());

      // Estimate point normals
      normal_estimation.setSearchMethod (tree);
      normal_estimation.setInputCloud (cloud_filtered);
      normal_estimation.setKSearch (50);
      normal_estimation.compute (*cloud_normals);

      // Create the segmentation object for the planar model and set all the parameters
      segmentation_from_normals.setOptimizeCoefficients (true);
      segmentation_from_normals.setModelType (pcl::SACMODEL_NORMAL_PLANE);
      segmentation_from_normals.setNormalDistanceWeight (0.1);
      segmentation_from_normals.setMethodType (pcl::SAC_RANSAC);
      segmentation_from_normals.setMaxIterations (100);
      segmentation_from_normals.setDistanceThreshold (0.03);
      segmentation_from_normals.setInputCloud (cloud_filtered);
      segmentation_from_normals.setInputNormals (cloud_normals);
      // Obtain the plane inliers and coefficients
      segmentation_from_normals.segment (*inliers_plane, *coefficients_plane);
      std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

      // Extract the planar inliers from the input cloud
      extract_indices.setInputCloud (cloud_filtered);
      extract_indices.setIndices (inliers_plane);
      extract_indices.setNegative (false);
      extract_indices.filter (*cloud_plane);

      // Remove the planar inliers, extract the rest
      extract_indices.setNegative (true);
      extract_indices.filter (*cloud_filtered);
      extract_normals.setNegative (true);
      extract_normals.setInputCloud (cloud_normals);
      extract_normals.setIndices (inliers_plane);
      extract_normals.filter (*cloud_normals);

      // Create the segmentation object for sphere segmentation and set all the parameters
      segmentation_from_normals.setOptimizeCoefficients (true);
      segmentation_from_normals.setModelType (pcl::SACMODEL_SPHERE);
      segmentation_from_normals.setMethodType (pcl::SAC_RANSAC);
      segmentation_from_normals.setNormalDistanceWeight (0.1);
      segmentation_from_normals.setMaxIterations (10000);
      segmentation_from_normals.setDistanceThreshold (0.05);
      segmentation_from_normals.setRadiusLimits (0, 0.1);
      segmentation_from_normals.setInputCloud (cloud_filtered);
      segmentation_from_normals.setInputNormals (cloud_normals);

      // Obtain the sphere inliers and coefficients
      segmentation_from_normals.segment (*inliers_sphere, *coefficients_sphere);
      std::cerr << "Sphere coefficients: " << *coefficients_sphere << std::endl;

      // Publish the sphere cloud
      extract_indices.setInputCloud (cloud_filtered);
      extract_indices.setIndices (inliers_sphere);
      extract_indices.setNegative (false);
      extract_indices.filter (*cloud_sphere);

      /*
      voxel_grid.setInputCloud (cloud_filtered);
      vox_grid.setLeafSize (0.02, 0.02, 0.02);
      vox_grid.filter (*cloud_filtered);
      ROS_INFO ("Point Cloud size after voxel grid: %zu", cloud_filtered->points.size ());
  */

      msg->header.frame_id = "/camera_depth_optical_frame";
      msg->height = 1;
      msg->width = cloud_sphere->points.size ();
      msg->points = cloud_sphere->points;
      msg->header.stamp = ros::Time::now ();
      pub.publish (msg);
      ros::spinOnce ();
      loop_rate.sleep ();
    }
}
