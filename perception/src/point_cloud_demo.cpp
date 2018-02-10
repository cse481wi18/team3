#include "perception/crop.h"
#include "perception/downsample.h"
#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

using namespace perception;

int main(int argc, char **argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  // publishign cropped publisher
  ros::Publisher crop_pub =
    nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  //Cropper cropper(crop_pub);
  // note we are subscribing to cloud_in as opposed to a known point cloud topic.
  // We can always remap cloud_in to a real topic name later.
  //ros::Subscriber sub =
  //  nh.subscribe("cloud_in", 1, &Cropper::Callback, &cropper);
  Downsampler downsampler(crop_pub);
  ros::Subscriber sub =
    nh.subscribe("cloud_in", 1, &Downsampler::Callback, &downsampler);

  ros::Publisher table_pub =
    nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  ros::Publisher marker_pub =
    nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  perception::Segmenter segmenter(table_pub, marker_pub);
  ros::Subscriber sub2 =
    nh.subscribe("cloud_in", 1, &perception::Segmenter::Callback, &segmenter); 

  ros::spin();
  return 0;
}
