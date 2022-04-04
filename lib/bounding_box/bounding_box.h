#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <cmath>
#include <limits>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_rviz_plugins/PictogramArray.h>

class BoundingBox
{
public:
    BoundingBox();
    BoundingBox(ros::NodeHandle pnh);
    ~BoundingBox();

    void SetCloud(std_msgs::Header header, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, bool in_estimate_pose);
    void getBoundingBox(std_msgs::Header header,
                        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &points_vector,
                        autoware_msgs::CloudClusterArray &inOutClusters);
    void ToROSMessage(std_msgs::Header header, autoware_msgs::CloudCluster &outClusterMessage);
    // void checkAllForMerge(std_msgs::Header header, std::vector<BoundingBoxPtr> &in_clusters, std::vector<BoundingBoxPtr> &out_clusters,
    //                                    double in_merge_threshold);
    // void checkClusterMerge(std_msgs::Header header, size_t in_cluster_id, std::vector<BoundingBoxPtr> &in_clusters,
    //                        std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
    //                        double in_merge_threshold);
    // void mergeClusters(std_msgs::Header header, const std::vector<BoundingBoxPtr> &in_clusters, std::vector<BoundingBoxPtr> &out_clusters,
    //                    std::vector<size_t> in_merge_indices, const size_t &current_index,
    //                    std::vector<bool> &in_out_merged_clusters);

    jsk_recognition_msgs::BoundingBox GetBoundingBox();
    /* \brief Returns the calculated PolygonArray of the object */
    geometry_msgs::PolygonStamped GetPolygon();

    bool inEstimatePose_;

    bool validCluster_;
    pcl::PointXYZ centroid_;
    pcl::PointXYZ minPoint_;
    pcl::PointXYZ maxPoint_;
    pcl::PointXYZ averagePoint_;
    jsk_recognition_msgs::BoundingBox boundingBox_;
    geometry_msgs::PolygonStamped polygon_;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud_(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud_;

    Eigen::Matrix3f eigenVectors_;
    Eigen::Vector3f eigenValues_;
    double orientation_angle_;

    double clusterMergeThreshold_;
};

typedef boost::shared_ptr<BoundingBox> BoundingBoxPtr;

#endif
