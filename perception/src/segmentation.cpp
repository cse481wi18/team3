#include "perception/segmentation.h"
#include "perception/object.h"
#include "perception/box_fitter.h"
#include <iostream>
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "pcl/common/common.h"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"
#include "pcl/segmentation/extract_clusters.h"
#include "shape_msgs/SolidPrimitive.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
  void SegmentTabletopScene(PointCloudC::Ptr cloud, std::vector<Object> *objects) {
    // TODO: Same as callback, but with visualization code removed.
  }

  void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coeff) {
    // I didn't realize this was already supposed to be done
    //std::cerr << "Before filtering, there are " << cloud->size() << " points" << std::endl;
    //// filter points that are lowest in Z coordinate (i.e. floor)
    //PointCloudC::Ptr cloud_filtered (new pcl::PointCloud<PointC>); 
    //pcl::PassThrough<PointC> pass;
    //pass.setInputCloud(cloud);
    //pass.setFilterFieldName("x");
    //pass.setFilterLimits(0.71, 1);
    //pass.filter(*cloud_filtered);
    //std::cerr << "After filtering, there are " << cloud_filtered->size() << " points" << std::endl;

    pcl::PointIndices indices_internal;
    pcl::SACSegmentation<PointC> seg;
    seg.setOptimizeCoefficients(true);

    // Search for a plane perpendicular to some axis (specified below).
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);

    // Set the distance to the plane for a point to be an inlier.
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    // seg.setInputCloud(cloud_filtered);

    // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
    Eigen::Vector3f axis;
    axis << 0, 0, 1;
    seg.setAxis(axis);
    seg.setEpsAngle(pcl::deg2rad(10.0));

    // coeff contains the coefficients of the plane:
    // ax + by + cz + d = 0
    // pcl::ModelCoefficients coeff;
    seg.segment(indices_internal, *coeff);

    double distance_above_plane;
    ros::param::param("distance_above_plane", distance_above_plane, 0.015);

    // Build custom indices that ignores points above the plane.
    for (size_t i = 0; i < cloud->size(); ++i) {
      const PointC& pt = cloud->points[i];
      float val = (*coeff).values[0] * pt.x + (*coeff).values[1] * pt.y +
                  (*coeff).values[2] * pt.z + (*coeff).values[3];
      if (val <= distance_above_plane) {
        indices->indices.push_back(i);
      }
    }
    // Commented out because custom built indices.
    //*indices = indices_internal;

    if (indices->indices.size() == 0) {
      ROS_ERROR("Unable to find surface.");
      return;
    }
  }

  Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub, const ros::Publisher& above_surface_pub)
    : surface_points_pub_(surface_points_pub), marker_pub_(marker_pub), above_surface_pub_(above_surface_pub) {}

  void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
    PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud_unfiltered);
    PointCloudC::Ptr cloud(new PointCloudC());
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

    std::vector<Object> objects;
    SegmentTabletopScene(cloud, &objects);
    for (size_t i = 0; i < objects.size(); ++i) {
      const Object& object = objects[i];

      // Publish a bounding box around it.
      visualization_msgs::Marker object_marker;
      object_marker.ns = "objects";
      object_marker.id = i;
      object_marker.header.frame_id = "base_link";
      object_marker.type = visualization_msgs::Marker::CUBE;
      object_marker.pose = object.pose;
      object_marker.scale = object.dimensions;
      object_marker.color.g = 1;
      object_marker.color.a = 0.3;
      marker_pub_.publish(object_marker);
    }
    pcl::fromROSMsg(msg, *cloud);
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
    // std::cerr << "Before segmenting, we have " << cloud->size() << " points" << std::endl;
    // SegmentSurface(cloud, table_inliers);
    // std::cerr << "After segmenting, we have " << cloud->size() << " points" << std::endl;
    // std::cerr << "After segmenting, we have " << table_inliers->indices.size() << " table inliers" << std::endl;

    // Extract subset of original_cloud into subset_cloud:
    // this is the subset that is *in* the table
    PointCloudC::Ptr subset_cloud(new PointCloudC());
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(table_inliers);
    extract.filter(*subset_cloud);

    sensor_msgs::PointCloud2 out_cloud;
    pcl:toROSMsg(*subset_cloud, out_cloud);
    surface_points_pub_.publish(out_cloud);

    // Publish bounding box (red table in the visualization)
    //visualization_msgs::Marker table_marker; // table_pose
    //table_marker.ns = "table";
    //table_marker.header.frame_id = "base_link";
    //table_marker.type = visualization_msgs::Marker::CUBE;
    //shape_msgs::SolidPrimitive shape;
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    SegmentSurface(cloud, table_inliers, coeff);
    //PointCloudC::Ptr extract_out(new PointCloudC());
    //perception::FitBox(*subset_cloud, coeff, *extract_out, shape, table_marker.pose);
    //if (shape.type == shape_msgs::SolidPrimitive::BOX) {
    //  table_marker.scale.x = shape.dimensions[0];
    //  table_marker.scale.y = shape.dimensions[1];
    //  table_marker.scale.z = shape.dimensions[2];
    //}
     //GetAxisAlignedBoundingBox(subset_cloud, &table_marker.pose, &table_marker.scale);
    //table_marker.color.r = 1;
    //table_marker.color.a = 0.8;
    //marker_pub_.publish(table_marker);

    // Segment surface objects.
    PointCloudC::Ptr cloud_out(new PointCloudC());
    extract.setNegative(true);
    extract.filter(*cloud_out);
    std::cerr << "The good cloud has " << cloud_out->size() << " points since the input has " << cloud->size() << " points and we remove " << table_inliers->indices.size() << " points. " << std::endl;
    // list of point clusters for each segmented object
    std::vector<pcl::PointIndices> object_indices;
    SegmentSurfaceObjects(cloud, table_inliers, &object_indices);

    // publish all above-surface points
    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud_out, msg_out);
    above_surface_pub_.publish(msg_out);

    for (size_t i = 0; i < object_indices.size(); ++i) {
      // Reify indices into a point cloud of the object.
      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      *indices = object_indices[i];
      PointCloudC::Ptr object_cloud(new PointCloudC());
      // TODO: fill in object_cloud using indices
      for (int j = 0; j < indices->indices.size(); j++) {
        pcl::PointXYZRGB p = cloud->at((*indices).indices[j]);
        object_cloud->push_back(p);
      }

      // Publish a bounding box around it.
      visualization_msgs::Marker object_marker;
      object_marker.ns = "objects";
      object_marker.id = i;
      object_marker.header.frame_id = "base_link";
      object_marker.type = visualization_msgs::Marker::CUBE;
      //GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
      //                          &object_marker.scale);
      shape_msgs::SolidPrimitive shape;
      geometry_msgs::Pose table_pose;
      PointCloudC::Ptr extract_out(new PointCloudC());
      perception::FitBox(*object_cloud, coeff, *extract_out, shape, table_pose);
      if (shape.type == shape_msgs::SolidPrimitive::BOX) {
        object_marker.scale.x = shape.dimensions[0];
        object_marker.scale.y = shape.dimensions[1];
        object_marker.scale.z = shape.dimensions[2];
      }
      object_marker.color.g = 1;
      object_marker.color.a = 0.3;
      // draw a green box around each segmented object
      marker_pub_.publish(object_marker);
    }
  }

  void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                             pcl::PointIndices::Ptr surface_indices,
                             std::vector<pcl::PointIndices>* object_indices) {
    // computes (cloud - surface_indices) and stores it in above_surface_indices
    pcl::ExtractIndices<PointC> extract;
    pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
    extract.setInputCloud(cloud);
    extract.setIndices(surface_indices);
    extract.setNegative(true);
    extract.filter(above_surface_indices->indices);
    std::cerr << "The bad cloud has " << above_surface_indices->indices.size() << " points since the input has " << cloud->size() << " points and we remove " << surface_indices->indices.size() << " points. " << std::endl;

    ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

    double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
    ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
    ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(cloud);
    euclid.setIndices(above_surface_indices);
    euclid.setClusterTolerance(cluster_tolerance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
    euclid.extract(*object_indices);
    // Find the size of the smallest and the largest object,
    // where size = number of points in the cluster
    size_t min_size = std::numeric_limits<size_t>::max();
    size_t max_size = std::numeric_limits<size_t>::min();
    for (size_t i = 0; i < object_indices->size(); ++i) {
      // TODO: implement this
      size_t cluster_size = object_indices->at(i).indices.size(); // holds int32s
      if (cluster_size < min_size) {
        min_size = cluster_size;
      }
      if (cluster_size > max_size) {
        max_size = cluster_size;
      }
    }

    ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
             object_indices->size(), min_size, max_size);
  }

  void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                 geometry_msgs::Pose* pose,
                                 geometry_msgs::Vector3* dimensions) {
    PointC min_pcl;
    PointC max_pcl;
    pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);

    pose->position.x = (min_pcl.x + max_pcl.x) / 2;
    pose->position.y = (min_pcl.y + max_pcl.y) / 2;
    pose->position.z = (min_pcl.z + max_pcl.z) / 2;
    pose->orientation.x = 0;
    pose->orientation.y = 0;
    pose->orientation.z = 0;
    pose->orientation.w = 1;

    dimensions->x = max_pcl.x - min_pcl.x;
    dimensions->y = max_pcl.y - min_pcl.y;
    dimensions->z = max_pcl.z - min_pcl.z;
  }
}  // namespace perception
