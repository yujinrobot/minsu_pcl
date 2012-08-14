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
sensor_msgs::PointCloud2::Ptr downsampled,output;
pcl::PointCloud<pcl::PointXYZ>::Ptr output_p, downsampled_XYZ;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
   // Do some downsampling to the point cloud
   pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
   sor.setInputCloud (cloud);
   sor.setLeafSize (0.01f, 0.01f, 0.01f);
   sor.filter (*downsampled);

   // Change from type sensor_msgs::PointCloud2 to pcl::PointXYZ
   pcl::fromROSMsg (*downsampled, *downsampled_XYZ);


   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
   // Create the segmentation object
   pcl::SACSegmentation<pcl::PointXYZ> seg;
   // Optional
   seg.setOptimizeCoefficients (true);
   // Mandatory
   seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setMaxIterations (1000);
   seg.setDistanceThreshold (0.01);

   // Create the filtering object
   pcl::ExtractIndices<pcl::PointXYZ> extract;

   // Segment the largest planar component from the cloud
   seg.setInputCloud (downsampled_XYZ);
   seg.segment (*inliers, *coefficients);
   if (inliers->indices.size () == 0)
   {
     std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
   }
   // Extract the inliers
   extract.setInputCloud (downsampled_XYZ);
   extract.setIndices (inliers);
   extract.setNegative (false);
   extract.filter (*output_p);
   std::cerr << "PointCloud representing the planar component: " << output_p->width * output_p->height << " data points." << std::endl;

   // Create the filtering object
   // extract.setNegative (true);
   // extract.filter (*cloud_f);
   // cloud_filtered.swap (cloud_f);


   pcl::toROSMsg (*output_p, *output);
   //Publish the results
   pub.publish(output);
}




int
main (int argc, char** argv)
{
 // INITIALIZE ROS
   ros::init (argc, argv, "table");
   ros::NodeHandle nh;
   ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, callback);
   pub = nh.advertise<sensor_msgs::PointCloud2> ("table", 1);

   ros::spin();

   return (0);
}
