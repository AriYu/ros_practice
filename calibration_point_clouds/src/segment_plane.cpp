#include <segment_plane.hpp>

SegmentPlane::SegmentPlane(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh), rate_(n.param("loop_rate", 10)),
    frame_id_(n.param<std::string>("inliers_pc_frame_id", "/inliers_pc_frame"))
{
  leaf_size_x_ = n.param("leaf_size_x", 0.3);
  leaf_size_y_ = n.param("leaf_size_y", 0.3);
  leaf_size_z_ = n.param("leaf_size_z", 0.3);
  
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/source_pointcloud"), 1, &SegmentPlane::detectPlaneCallback, this);
  inliers_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(n.param<std::string>("inliers_pc_topic_name", "/inliers_pointcloud"), 1);
}

void SegmentPlane::detectPlaneCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_source;
  pcl::fromROSMsg(*source_pc, pcl_source);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_source_ptr(new pcl::PointCloud<pcl::PointXYZ>(pcl_source));
  pcl::PointCloud<pcl::PointXYZ>::Ptr resized_pc_ptr (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (leaf_size_x_, leaf_size_y_, leaf_size_z_);
  approximate_voxel_filter.setInputCloud (pcl_source_ptr);
  approximate_voxel_filter.filter (*resized_pc_ptr);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.1);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  
  seg.setInputCloud(resized_pc_ptr);
  seg.segment(*inliers, *coefficients);

  if(inliers->indices.size() == 0)
    {
      ROS_WARN("Could not estimate a planar model for the given dataset");
      return;
    }

  extract.setInputCloud(resized_pc_ptr);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_p);
  
  sensor_msgs::PointCloud2 inliers_pc;
  pcl::toROSMsg(*cloud_p, inliers_pc);
  inliers_pc.header.stamp = ros::Time::now();
  inliers_pc.header.frame_id = frame_id_;
  inliers_pc_pub_.publish(inliers_pc);
}

void SegmentPlane::run()
{
  while(nh_.ok())
    {
      ros::spinOnce();
      rate_.sleep();
    }
}
