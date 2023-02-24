
/*
 * @Author: xiaohu
 * @Date: 2022-04-02 00:26:55
 * @Last Modified by: xiaohu
 * @Last Modified time: 2022-04-02 01:12:59
 */
#include "voxel_grid_filter.h"

VoxelGridFilter::VoxelGridFilter() {}
VoxelGridFilter::VoxelGridFilter(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{
  /* Initialize tuning parameter */
  private_node_handle.param("is_downsample", isDownsample, true);
  private_node_handle.param("leaf_size", leafSize, 0.1);
}

VoxelGridFilter::~VoxelGridFilter() {}

void VoxelGridFilter::downsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud)
{
  // // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
  if (leafSize >= 0.1)
  {
    // Downsampling the velodyne scan using VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(in_cloud);
    //参数为float
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    voxel.filter(*out_cloud);
  }
  else
  {
    out_cloud = in_cloud;
  }
}
