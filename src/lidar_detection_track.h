/*
 * @Author: zhao_xiaohu
 * @Date: 2022-04-02 00:26:55
 * @Last Modified by: zhao_xiaohu
 * @Last Modified time: 2022-04-02 01:12:59
 */

#ifndef LIDAR_DETECTION_TRACK_H_
#define LIDAR_DETECTION_TRACK_H_

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>

#include "../lib/ground_detector/patchwork/patchwork.h"
#include "../lib/visualization/visualize_detected_objects.h"
#include "../lib/pre_process/roi_clip/roi_clip.h"
#include "../lib/cluster/euclideanCluster/euclideanCluster.h"
#include "../lib/pre_process/voxel_grid_filter/voxel_grid_filter.h"
#include "../lib/bounding_box/bounding_box.h"
#include "../lib/visualization/visualize_detected_objects.h"

class lidarPerception
{
public:
    lidarPerception(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~lidarPerception(){};

    RoiClip roiClip_;
    VoxelGridFilter voxelGridFilter_;
    PatchWork patchWork_;
    EuclideanCluster cluster_;

    BoundingBox boundingBox_;
    VisualizeDetectedObjects vdo_;

    ros::Publisher _pub_clip_cloud;
    ros::Publisher _pub_ground_cloud;
    ros::Publisher _pub_noground_cloud;
    ros::Publisher _pub_cluster_cloud;
    ros::Publisher _pub_clusters_message;

    ros::Publisher _pub_detected_objects;
    ros::Publisher _pub_detected_3Dobjects;

    ros::Publisher _pub_cluster_visualize_markers;
    ros::Publisher _pub_3Dobjects_visualize_markers;

    VisualizeDetectedObjects vdto;

    // lidar::PointPillars pointpillars;

private:
    void ClusterCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);
    // void pointPillarsCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);
    void publishDetectedObjects(const autoware_msgs::CloudClusterArray &in_clusters, autoware_msgs::DetectedObjectArray &detected_objects);
    // void Bbox3DToObjectArray(std_msgs::Header header, std::vector<lidar::Bbox3D> &boxes,autoware_msgs::DetectedObjectArray &detected_objects);
};

#endif