
#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "Eigen/Dense"

/*
 * Eigen is roughly the C++ equivalent of Python's numpy.
 */

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

using namespace perception;

Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {}

void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  // Converting point cloud received from ROS to PCL's representation
  //  of point cloud
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  // lets us change value using `rosparam set [param] [value]`
  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param("crop_min_x", min_x, -100.0);
  ros::param::param("crop_max_x", max_x, 100.0);
  ros::param::param("crop_min_y", min_y, -100.0);
  ros::param::param("crop_max_y", max_y, 100.0);
  ros::param::param("crop_min_z", min_z, 0.5);
  ros::param::param("crop_max_z", max_z, 1.5);
  Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
  Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
  // crop the pointcloud
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud);
  crop.setMin(min_pt);
  crop.setMax(max_pt);
  crop.filter(*cropped_cloud);
  ROS_ERROR("Cropped to %ld points", cropped_cloud->size());

  // publishing cropped pointcloud by converting pcl to ROS representation
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cropped_cloud, msg_out);
  pub_.publish(msg_out);
}
