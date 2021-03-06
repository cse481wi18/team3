#include <vector>
#include <iostream>
#include "perception/crop.h"
#include "perception/downsample.h"
#include "perception/segmentation.h"
#include "perception/object_recognizer.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "perception_msgs/ObjectFeatures.h"

using namespace perception;

int main(int argc, char **argv) {
  ros::init(argc, argv, "point_cloud_demo");
  if (argc < 2) {
    ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
    ros::spinOnce();
  }
  std::string data_dir(argv[1]);
  ros::NodeHandle nh;
  // publishign cropped publisher
  ros::Publisher crop_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  ros::Publisher downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_downsampled_cloud", 1, true);

  Cropper cropper(crop_pub);
  Downsampler downsampler(downsample_pub);

  ros::Subscriber crop_sub =
    nh.subscribe("cloud_in", 1, &Cropper::Callback, &cropper);
  ros::Subscriber sub =
    nh.subscribe("cropped_cloud", 1, &Downsampler::Callback, &downsampler);

  ros::Publisher table_pub =
    nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  ros::Publisher marker_pub =
    nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  ros::Publisher above_surface_pub =
    nh.advertise<sensor_msgs::PointCloud2>("above_surface", 1, true);

  // Create the object recognizer
  std::vector<perception_msgs::ObjectFeatures> dataset;
  perception::LoadData(data_dir, &dataset);
  perception::ObjectRecognizer recognizer(dataset);

  perception::Segmenter segmenter(table_pub, marker_pub, above_surface_pub, recognizer);

  ros::Subscriber sub2 =
    nh.subscribe("cropped_downsampled_cloud", 1, &perception::Segmenter::Callback, &segmenter); 

  ros::spin();
  return 0;
}
