

#include "euclideanCluster.h"

EuclideanCluster::EuclideanCluster(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    pnh.param("clusterTolerance", clusterTolerance_, 0.35);
    pnh.param("minClusterSize", minClusterSize_, 5);
    pnh.param("maxClusterSize", maxClusterSize_, 20000);
    pnh.param("use_multiple_thres", use_multiple_thres_, true);

    clustering_distances_ = {0.5, 1.1, 1.6, 2.1, 2.6};
    clustering_ranges_ = {15, 30, 45, 60};
}
void EuclideanCluster::cluster_vector(const pcl::PointCloud<pcl::PointXYZI>::Ptr in, std::vector<pcl::PointIndices> &clusters)
{
    //设置查找方式－kdtree
    pcl::search::Search<pcl::PointXYZI>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZI>>(new pcl::search::KdTree<pcl::PointXYZI>);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(clusterTolerance_); // 2cm
    ec.setMinClusterSize(minClusterSize_);     // 100
    ec.setMaxClusterSize(maxClusterSize_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in);
    ec.extract(clusters);
}

void EuclideanCluster::segmentByDistance(const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                         pcl::PointCloud<pcl::PointXYZI>::Ptr &outCloudPtr, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &points_vector)
{
    // cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
    // in this way, the points farther in the pc will also be clustered
    std::vector<pcl::PointIndices> clusterIndices;
    if (!use_multiple_thres_)
    {

        cluster_vector(in, clusterIndices);

        for (auto it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*in, it->indices, *temp_cluster);
            *outCloudPtr += *temp_cluster;
            points_vector.push_back(temp_cluster);
        }
    }
    else
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_segments_array(7);
        for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            cloud_segments_array[i] = tmp_cloud;
        }

        for (unsigned int i = 0; i < in->points.size(); i++)
        {
            pcl::PointXYZI current_point;
            current_point.x = in->points[i].x;
            current_point.y = in->points[i].y;
            current_point.z = in->points[i].z;
            current_point.intensity = in->points[i].intensity;

            float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

            if (origin_distance < clustering_ranges_[0])
            {
                cloud_segments_array[0]->points.push_back(current_point);
            }
            else if (origin_distance < clustering_ranges_[1])
            {
                cloud_segments_array[1]->points.push_back(current_point);
            }
            else if (origin_distance < clustering_ranges_[2])
            {
                cloud_segments_array[2]->points.push_back(current_point);
            }
            else if (origin_distance < clustering_ranges_[3])
            {
                cloud_segments_array[3]->points.push_back(current_point);
            }
            else
            {
                cloud_segments_array[4]->points.push_back(current_point);
            }
        }

        std::vector<std::thread> thread_vec(cloud_segments_array.size());
        for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
        {
            std::promise<std::vector<pcl::PointIndices>> promiseObj;
            std::shared_future<std::vector<pcl::PointIndices>> futureObj = promiseObj.get_future();
            thread_vec[i] = std::thread(&EuclideanCluster::clusterIndicesMultiThread, this, cloud_segments_array[i], std::ref(clustering_distances_[i]), std::ref(promiseObj));
            clusterIndices = futureObj.get();
            for (int j = 0; j < clusterIndices.size(); j++)
            {
                //每次聚类得出的indices为输入点云对应的索引
                pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cluster(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::copyPointCloud(*cloud_segments_array[i], clusterIndices[j], *temp_cluster);
                *outCloudPtr += *temp_cluster;
                points_vector.push_back(temp_cluster);
            }
        }
        for (int i = 0; i < thread_vec.size(); i++)
        {
            thread_vec[i].join();
        }
    }
}

void EuclideanCluster::clusterIndicesMultiThread(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double in_max_cluster_distance,
                                                 std::promise<std::vector<pcl::PointIndices>> &promiseObj)
{
    // make it flat
    // for (size_t i = 0; i < cloud_2d->points.size(); i++)
    // {
    //     cloud_2d->points[i].z = 0;
    // }
    pcl::search::Search<pcl::PointXYZI>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZI>>(new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(in_max_cluster_distance); // 2cm
    ec.setMinClusterSize(minClusterSize_);           // 100
    ec.setMaxClusterSize(maxClusterSize_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in_cloud_ptr);
    ec.extract(indices);

    promiseObj.set_value(indices);
}
