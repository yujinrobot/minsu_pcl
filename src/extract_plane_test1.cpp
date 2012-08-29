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
 *  cloud -> passthrough filter -> SACSegmentation -> RANSAC -> plane
 *
 *  handling of pass through filter
 *  we set a boundary value about floor.
 *
 *  subscribe topic : /camera/depth/points
 *
 *
 *  pcl::SACSegmentation
 *
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ros::Publisher passthrough_pub;
ros::Publisher rest_whole_pub;
ros::Publisher rest_ball_candidate_pub;
ros::Publisher plane_pub;
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
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices2;

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // The plane and sphere coefficients
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients ());
  pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients ());

  // The plane and sphere inliers
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices ());
  pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices ());

  // The point clouds
  sensor_msgs::PointCloud2::Ptr passthrough_filtered (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr plane_seg_output_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr sphere_RANSAC_output_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr rest_whole_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr rest_cloud_filtered (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr sphere_output_cloud (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr whole_pc (new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr ball_candidate_output_cloud (new sensor_msgs::PointCloud2);

  // The PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_output (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr remove_plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_output (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_RANSAC_output (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr remove_false_ball_candidate (new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> inliers;

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
  pass.setFilterLimits (0, 2.5);
  pass.filter (*passthrough_filtered);

  passthrough_pub.publish(passthrough_filtered);

  ros::Time pass_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
   * for plane features pcl::SACSegmentation
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ros::Time plane_seg_start = ros::Time::now();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*passthrough_filtered, *plane_seg_cloud);

  // Optional
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setAxis(Eigen::Vector3f (0, -1, 0));       // best plane should be perpendicular to z-axis
  seg.setMaxIterations (40);
  seg.setDistanceThreshold (0.05);
  seg.setInputCloud (plane_seg_cloud);
  seg.segment (*inliers_plane, *coefficients_plane);

  extract_indices.setInputCloud(plane_seg_cloud);
  extract_indices.setIndices(inliers_plane);
  extract_indices.setNegative(false);
  extract_indices.filter(*plane_seg_output);


  pcl::toROSMsg (*plane_seg_output, *plane_seg_output_cloud);
  plane_pub.publish(plane_seg_output_cloud);

  ros::Time plane_seg_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*
   * Extract rest plane and passthrough filtering
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ros::Time rest_pass_start = ros::Time::now();

  // Create the filtering object
  // Remove the planar inliers, extract the rest
  extract_indices.setNegative (true);
  extract_indices.filter (*remove_plane_cloud);
  plane_seg_cloud.swap (remove_plane_cloud);

  // publish result of Removal the planar inliers, extract the rest
  pcl::toROSMsg (*plane_seg_cloud, *rest_whole_cloud);
  rest_whole_pub.publish(rest_whole_cloud);          // 'rest_whole_cloud' substituted whole_pc

  ros::Time rest_pass_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ros::Time find_ball_start = ros::Time::now();
  int iter = 0;
  bool BALL = false;

  while(iter < 5)
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*
     * for sphere features pcl::SACSegmentation
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::fromROSMsg (*rest_whole_cloud, *sphere_cloud);

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
       std::cerr << "[[--Can't find the Sphere Candidate.--]]" << std::endl;
    else {
      extract_indices.setInputCloud(sphere_cloud);
      extract_indices.setIndices(inliers_sphere);
      extract_indices.setNegative(false);
      extract_indices.filter(*sphere_output);
      pcl::toROSMsg (*sphere_output, *sphere_output_cloud);
      sphere_seg_pub.publish(sphere_output_cloud);          // 'sphere_output_cloud' means ball candidate point cloud
    }

    ros::Time sphere_end = ros::Time::now();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*
     * for sphere features pcl::SampleConsensusModelSphere
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ros::Time sphere_RANSAC_start = ros::Time::now();

    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (sphere_output));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    // copies all inliers of the model computed to another PointCloud
    pcl::copyPointCloud<pcl::PointXYZ>(*sphere_output, inliers, *sphere_RANSAC_output);

    pcl::toROSMsg (*sphere_RANSAC_output, *sphere_RANSAC_output_cloud);
    //sphere_RANSAC_pub.publish(sphere_RANSAC_output_cloud);


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*
     * To discriminate a ball
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    double w = 0;
    w = double(sphere_RANSAC_output_cloud->width * sphere_RANSAC_output_cloud->height)
        /double(sphere_output_cloud->width * sphere_output_cloud->height);

    std::cout << "w : " << w << std::endl;

    if (w > 0.9) {
      //BALL = true;
      std::cout << "can find a ball" << std::endl;
      sphere_RANSAC_pub.publish(sphere_RANSAC_output_cloud);
      break;
    } else {
      //BALL = false;
      std::cout << "can not find a ball" << std::endl;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /*
       * Exclude false ball candidate
       */
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      //ros::Time rest_pass_start = ros::Time::now();

      // Create the filtering object
      // Remove the planar inliers, extract the rest
      extract_indices2.setInputCloud(plane_seg_cloud);
      extract_indices2.setIndices(inliers_sphere);
      extract_indices2.setNegative (true);
      extract_indices2.filter (*remove_false_ball_candidate);
      sphere_RANSAC_output.swap (remove_false_ball_candidate);

      // publish result of Removal the planar inliers, extract the rest
      pcl::toROSMsg (*sphere_RANSAC_output, *ball_candidate_output_cloud);
      rest_ball_candidate_pub.publish(ball_candidate_output_cloud);
      rest_whole_cloud = ball_candidate_output_cloud;

    }
    iter++;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ros::Time sphere_RANSAC_end = ros::Time::now();

  }
  ros::Time find_ball_end = ros::Time::now();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  ros::Time whole_end = ros::Time::now();

  std::cout << "cloud size             : " << cloud->width * cloud->height << std::endl;
  std::cout << "plane size             : " << plane_seg_output_cloud->width * plane_seg_output_cloud->height << std::endl;
  std::cout << "rest size              : " << rest_whole_cloud->width * rest_whole_cloud->height << std::endl;
  std::cout << "sphere size            : " << sphere_output_cloud->width * sphere_output_cloud->height << std::endl;
  std::cout << "sphere RANSAC size     : " << sphere_RANSAC_output_cloud->width * sphere_RANSAC_output_cloud->height << "   " << sphere_RANSAC_output->points.size() << std::endl;
  std::cout << "sphereness             : " << double(sphere_RANSAC_output_cloud->width * sphere_RANSAC_output_cloud->height)
                                              /double(sphere_output_cloud->width * sphere_output_cloud->height) << std::endl;

  std::cout << "model coefficient      : " << coefficients_plane->values[0] << " " 
				           << coefficients_plane->values[1] << " " 
				           << coefficients_plane->values[2] << " " 
                                           << coefficients_plane->values[3] << " " << std::endl; 
                                        


  printf("\n");

  std::cout << "whole time             : " << whole_end - whole_start << " sec" << std::endl;
  std::cout << "declare types time     : " << declare_types_end - declare_types_start << " sec" << std::endl;
  std::cout << "passthrough time       : " << pass_end - pass_start << " sec" << std::endl;
  std::cout << "plane time             : " << plane_seg_end - plane_seg_start << " sec" << std::endl;
  std::cout << "rest and pass time     : " << rest_pass_end - rest_pass_start << " sec" << std::endl;
  std::cout << "find a ball time       : " << find_ball_end - find_ball_start << " sec" << std::endl;
  //std::cout << "sphere time            : " << sphere_end - sphere_start << " sec" << std::endl;
  //std::cout << "sphere ransac time     : " << sphere_RANSAC_end - sphere_RANSAC_start << " sec" << std::endl;
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
  plane_pub = nh.advertise<sensor_msgs::PointCloud2> ("plane_cloud", 1);
  rest_whole_pub = nh.advertise<sensor_msgs::PointCloud2> ("rest_whole_cloud", 1);
  sphere_seg_pub = nh.advertise<sensor_msgs::PointCloud2> ("sphere_cloud", 1);
  sphere_RANSAC_pub = nh.advertise<sensor_msgs::PointCloud2> ("sphere_RANSAC_cloud", 1);
  rest_ball_candidate_pub = nh.advertise<sensor_msgs::PointCloud2> ("rest_ball_candidate_cloud", 1);

  ros::spin();

  return (0);
}
