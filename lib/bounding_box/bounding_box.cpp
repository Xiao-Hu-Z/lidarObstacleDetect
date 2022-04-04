
#include "bounding_box.h"

void checkClusterMerge(std_msgs::Header header, size_t in_cluster_id, std::vector<BoundingBoxPtr> &in_clusters,
                       std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
                       double in_merge_threshold)
{
  // std::cout << "checkClusterMerge" << std::endl;
  pcl::PointXYZ point_a = in_clusters[in_cluster_id]->centroid_;
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    if (i != in_cluster_id && !in_out_visited_clusters[i])
    {
      pcl::PointXYZ point_b = in_clusters[i]->centroid_;
      double distance = sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.y - point_a.y, 2));
      if (distance <= in_merge_threshold)
      {
        in_out_visited_clusters[i] = true;
        out_merge_indices.push_back(i);
        // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
        checkClusterMerge(header, i, in_clusters, in_out_visited_clusters, out_merge_indices, in_merge_threshold);
      }
    }
  }
}

void mergeClusters(std_msgs::Header header, std::vector<BoundingBoxPtr> &in_clusters, std::vector<BoundingBoxPtr> &out_clusters,
                   std::vector<size_t> in_merge_indices, const size_t &current_index,
                   std::vector<bool> &in_out_merged_clusters, bool inEstimatePose_)
{
  // std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
  pcl::PointCloud<pcl::PointXYZI> mono_cloud;
  BoundingBoxPtr merged_cluster(new BoundingBox());
  for (size_t i = 0; i < in_merge_indices.size(); i++)
  {
    sum_cloud += *(in_clusters[in_merge_indices[i]]->pointCloud_);
    in_out_merged_clusters[in_merge_indices[i]] = true;
  }
  std::vector<int> indices(sum_cloud.points.size(), 0);
  for (size_t i = 0; i < sum_cloud.points.size(); i++)
  {
    indices[i] = i;
  }

  if (sum_cloud.points.size() > 0)
  {
    pcl::copyPointCloud(sum_cloud, mono_cloud);
    merged_cluster->SetCloud(header, mono_cloud.makeShared(), inEstimatePose_);
    out_clusters.push_back(merged_cluster);
  }
}

void checkAllForMerge(std_msgs::Header header, std::vector<BoundingBoxPtr> &in_clusters, std::vector<BoundingBoxPtr> &out_clusters,
                      float in_merge_threshold, bool inEstimatePose_)
{
  // std::cout << "checkAllForMerge" << std::endl;
  std::vector<bool> visited_clusters(in_clusters.size(), false);
  std::vector<bool> merged_clusters(in_clusters.size(), false);
  size_t current_index = 0;
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    if (!visited_clusters[i])
    {
      visited_clusters[i] = true;
      std::vector<size_t> merge_indices;
      checkClusterMerge(header, i, in_clusters, visited_clusters, merge_indices, in_merge_threshold);
      mergeClusters(header, in_clusters, out_clusters, merge_indices, current_index++, merged_clusters, inEstimatePose_);
    }
  }
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    // check for clusters not merged, add them to the output
    if (!merged_clusters[i])
    {
      out_clusters.push_back(in_clusters[i]);
    }
  }

  // ClusterPtr cluster(new Cluster());
}

BoundingBox::BoundingBox()
{
}

BoundingBox::BoundingBox(ros::NodeHandle pnh)
{
  pnh.param("inEstimatePose", inEstimatePose_, false);
  pnh.param("clusterMergeThreshold", clusterMergeThreshold_, 0.7);
}

BoundingBox::~BoundingBox() {}

void BoundingBox::SetCloud(std_msgs::Header header, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, bool in_estimate_pose)
{

  // extract pointcloud using the indices
  // calculate min and max points
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCluster(new pcl::PointCloud<pcl::PointXYZRGB>);
  float min_x = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_z = -std::numeric_limits<float>::max();
  float average_x = 0, average_y = 0, average_z = 0;

  for (int i = 0; i < in->points.size(); i++)
  {
    // fill new colored cluster point by point
    pcl::PointXYZRGB p;
    p.x = in->points[i].x;
    p.y = in->points[i].y;
    p.z = in->points[i].z;

    average_x += p.x;
    average_y += p.y;
    average_z += p.z;
    centroid_.x += p.x;
    centroid_.y += p.y;
    centroid_.z += p.z;
    currentCluster->points.push_back(p);

    if (p.x < min_x)
      min_x = p.x;
    if (p.y < min_y)
      min_y = p.y;
    if (p.z < min_z)
      min_z = p.z;
    if (p.x > max_x)
      max_x = p.x;
    if (p.y > max_y)
      max_y = p.y;
    if (p.z > max_z)
      max_z = p.z;
  }
  // min, max points
  minPoint_.x = min_x;
  minPoint_.y = min_y;
  minPoint_.z = min_z;
  maxPoint_.x = max_x;
  maxPoint_.y = max_y;
  maxPoint_.z = max_z;

  // calculate centroid, average
  if (in->points.size() > 0)
  {
    centroid_.x /= in->points.size();
    centroid_.y /= in->points.size();
    centroid_.z /= in->points.size();

    average_x /= in->points.size();
    average_y /= in->points.size();
    average_z /= in->points.size();
  }

  averagePoint_.x = average_x;
  averagePoint_.y = average_y;
  averagePoint_.z = average_z;

  // calculate bounding box
  float length_ = maxPoint_.x - minPoint_.x;
  float width_ = maxPoint_.y - minPoint_.y;
  float height_ = maxPoint_.z - minPoint_.z;

  boundingBox_.header = header;

  boundingBox_.pose.position.x = minPoint_.x + length_ / 2;
  boundingBox_.pose.position.y = minPoint_.y + width_ / 2;
  boundingBox_.pose.position.z = minPoint_.z + height_ / 2;

  boundingBox_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
  boundingBox_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
  boundingBox_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

  // pose estimation
  double rz = 0;

  std::vector<cv::Point2f> points;
  for (unsigned int i = 0; i < currentCluster->points.size(); i++)
  {
    cv::Point2f pt;
    pt.x = currentCluster->points[i].x;
    pt.y = currentCluster->points[i].y;
    points.push_back(pt);
  }

  std::vector<cv::Point2f> hull;
  cv::convexHull(points, hull);

  polygon_.header = header;
  for (size_t i = 0; i < hull.size() + 1; i++)
  {
    geometry_msgs::Point32 point;
    point.x = hull[i % hull.size()].x;
    point.y = hull[i % hull.size()].y;
    point.z = minPoint_.z;
    polygon_.polygon.points.push_back(point);
  }

  if (in_estimate_pose)
  {
    cv::RotatedRect box = minAreaRect(hull);
    rz = box.angle * 3.14 / 180;
    boundingBox_.pose.position.x = box.center.x;
    boundingBox_.pose.position.y = box.center.y;
    boundingBox_.dimensions.x = box.size.width;
    boundingBox_.dimensions.y = box.size.height;
  }

  // set bounding box direction
  tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);

  /** \brief convert Quaternion to Quaternion msg*/
  tf::quaternionTFToMsg(quat, boundingBox_.pose.orientation);

  currentCluster->width = currentCluster->points.size();
  currentCluster->height = 1;
  currentCluster->is_dense = true;

  // Get EigenValues, eigenvectors
  if (currentCluster->points.size() > 3)
  {
    pcl::PCA<pcl::PointXYZ> currentClusterPca;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cluster_mono(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*currentCluster, *current_cluster_mono);

    currentClusterPca.setInputCloud(current_cluster_mono);
    eigenVectors_ = currentClusterPca.getEigenVectors();
    eigenValues_ = currentClusterPca.getEigenValues();
  }

  validCluster_ = true;
  pointCloud_ = currentCluster;
}

void BoundingBox::getBoundingBox(std_msgs::Header header,
                                 std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &points_vector,
                                 autoware_msgs::CloudClusterArray &inOutClusters)
{
  std::vector<BoundingBoxPtr> Clusters;
  for (int i = 0; i < points_vector.size(); i++)
  {
    // pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::copyPointCloud(*in, it->indices, *temp_cluster);
    // *outCloudPtr += *temp_cluster;

    BoundingBoxPtr cluster(new BoundingBox());
    cluster->SetCloud(header, points_vector[i], inEstimatePose_);
    Clusters.push_back(cluster);
  }

  // Clusters can be merged or checked in here
  // check for mergable clusters
  std::vector<BoundingBoxPtr> midClusters;
  std::vector<BoundingBoxPtr> finalClusters;

  if (Clusters.size() > 0)
    checkAllForMerge(header, Clusters, midClusters, clusterMergeThreshold_, inEstimatePose_);
  else
    midClusters = Clusters;

  if (midClusters.size() > 0)
    checkAllForMerge(header, midClusters, finalClusters, clusterMergeThreshold_, inEstimatePose_);
  else
    finalClusters = midClusters;

  // Get final PointCloud to be published
  for (unsigned int i = 0; i < Clusters.size(); i++)
  {
    if (Clusters[i]->validCluster_)
    {
      autoware_msgs::CloudCluster cloudCluster;
      Clusters[i]->ToROSMessage(header, cloudCluster);
      inOutClusters.clusters.push_back(cloudCluster);
    }
  }

  inOutClusters.header = header;
}

void BoundingBox::ToROSMessage(std_msgs::Header header, autoware_msgs::CloudCluster &outClusterMessage)
{
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(*(this->pointCloud_), cloud_msg);
  cloud_msg.header = header;
  outClusterMessage.header = header;

  outClusterMessage.cloud = cloud_msg;
  outClusterMessage.min_point.header = header;
  outClusterMessage.min_point.point.x = this->minPoint_.x;
  outClusterMessage.min_point.point.y = this->minPoint_.y;
  outClusterMessage.min_point.point.z = this->minPoint_.z;

  outClusterMessage.max_point.header = header;
  outClusterMessage.max_point.point.x = this->maxPoint_.x;
  outClusterMessage.max_point.point.y = this->maxPoint_.y;
  outClusterMessage.max_point.point.z = this->maxPoint_.z;

  outClusterMessage.avg_point.header = header;
  outClusterMessage.avg_point.point.x = this->averagePoint_.x;
  outClusterMessage.avg_point.point.y = this->averagePoint_.y;
  outClusterMessage.avg_point.point.z = this->averagePoint_.z;

  outClusterMessage.centroid_point.header = header;
  outClusterMessage.centroid_point.point.x = this->centroid_.x;
  outClusterMessage.centroid_point.point.y = this->centroid_.y;
  outClusterMessage.centroid_point.point.z = this->centroid_.z;

  // outClusterMessage.estimated_angle = this->GetOrientationAngle();

  outClusterMessage.dimensions = this->boundingBox_.dimensions;

  outClusterMessage.bounding_box = this->boundingBox_;

  outClusterMessage.convex_hull = this->polygon_;

  Eigen::Vector3f eigen_values = this->eigenValues_;
  outClusterMessage.eigen_values.x = eigen_values.x();
  outClusterMessage.eigen_values.y = eigen_values.y();
  outClusterMessage.eigen_values.z = eigen_values.z();

  Eigen::Matrix3f eigen_vectors = this->eigenVectors_;
  for (unsigned int i = 0; i < 3; i++)
  {
    geometry_msgs::Vector3 eigen_vector;
    eigen_vector.x = eigen_vectors(i, 0);
    eigen_vector.y = eigen_vectors(i, 1);
    eigen_vector.z = eigen_vectors(i, 2);
    outClusterMessage.eigen_vectors.push_back(eigen_vector);
  }

  /*std::vector<float> fpfh_descriptor = GetFpfhDescriptor(8, 0.3, 0.3);
  out_cluster_message.fpfh_descriptor.data = fpfh_descriptor;*/
}
