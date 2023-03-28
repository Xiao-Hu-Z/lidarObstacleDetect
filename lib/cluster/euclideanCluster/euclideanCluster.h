

#ifndef EUCLIDEAN_CLUSTER_H_
#define EUCLIDEAN_CLUSTER_H_

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <future>

#include <ros/ros.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>

class EuclideanCluster
{
public:
  EuclideanCluster(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~EuclideanCluster(){};
  void cluster_vector(const pcl::PointCloud<pcl::PointXYZI>::Ptr in, std::vector<pcl::PointIndices> &clusters);
  void segmentByDistance(const pcl::PointCloud<pcl::PointXYZI>::Ptr in, pcl::PointCloud<pcl::PointXYZI>::Ptr &outCloudPtr,
                         std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &points_vector);

  void clusterIndicesMultiThread(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double in_max_cluster_distance,
                                 std::promise<std::vector<pcl::PointIndices>> &promiseObj);

private:
  double clusterTolerance_;
  int minClusterSize_;
  int maxClusterSize_;
  std::vector<double> clustering_distances_;
  std::vector<double> clustering_ranges_;
  bool use_multiple_thres_;

  std::mutex mutex_; //先定义互斥锁
};

#endif
