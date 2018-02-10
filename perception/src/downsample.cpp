
#include "perception/downsample.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "Eigen/Dense"

/*
 * Eigen is roughly the C++ equivalent of Python's numpy.
 */

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

using namespace perception;

Downsampler::Downsampler(const ros::Publisher& pub) : pub_(pub) {}

void Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {
  // Converting point cloud received from ROS to PCL's representation
  //  of point cloud
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  // Crop first.
  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud);
  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param("crop_min_x", min_x, 0.3);
  ros::param::param("crop_min_y", min_y, -1.0);
  ros::param::param("crop_min_z", min_z, 0.5);
  ros::param::param("crop_max_x", max_x, 0.9);
  ros::param::param("crop_max_y", max_y, 1.0);
  ros::param::param("crop_max_z", max_z, 1.5);
  Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
  Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
  crop.setMin(min_pt);
  crop.setMax(max_pt);
  crop.filter(*cropped_cloud);
  ROS_INFO("Cropped to %ld points", cropped_cloud->size());

  // TODO: NEED TO PUT THE FILTER BACK... OOPS! REFER TO CROP.CPP
  // I believe this is done -Kennan
  PointCloudC::Ptr downsampled_cloud(new PointCloudC());
  pcl::VoxelGrid<PointC> vox;
  vox.setInputCloud(cropped_cloud);
  double voxel_size;
  ros::param::param("voxel_size", voxel_size, 0.01);
  vox.setLeafSize(voxel_size, voxel_size, voxel_size);
  vox.filter(*downsampled_cloud);

  // publishing downsampled pointcloud by converting pcl to ROS representation
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*downsampled_cloud, msg_out);
  pub_.publish(msg_out);
  ROS_INFO("Downsampled to %ld points", downsampled_cloud->size());
}
