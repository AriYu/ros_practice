#ifndef SEGMENT_PLANE_H
#define SEGMENT_PLANE_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

class SegmentPlane
{
public:
  SegmentPlane(ros::NodeHandle nh, ros::NodeHandle n);
  void detectPlaneCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc);
  void run();
private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  std::string frame_id_;
  ros::Publisher inliers_pc_pub_;
  ros::Subscriber source_pc_sub_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_clouds_;
  
  double leaf_size_x_;
  double leaf_size_y_;
  double leaf_size_z_;
};

#endif /* SEGMENT_PLANE_H */
