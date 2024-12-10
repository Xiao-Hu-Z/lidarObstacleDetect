
#include "lidar_detection_track.h"

// 时间统计
int64_t euclidean_time = 0.;
int64_t total_time = 0.;
int counter = 0;



int64_t gtm()
{
	struct timeval tm;
	gettimeofday(&tm, 0);
	int64_t re = (((int64_t)tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
	return re;
}

void publishCloud(
	const ros::Publisher *in_publisher, std_msgs::Header header,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header = header;
	in_publisher->publish(cloud_msg);
}

lidarPerception::lidarPerception(ros::NodeHandle nh, ros::NodeHandle pnh)
	: roiClip_(nh, pnh), voxelGridFilter_(nh, pnh), cluster_(nh, pnh), boundingBox_(pnh)
{
	// nuscenes的 topic
	ros::Subscriber sub = nh.subscribe("/LIDAR_TOP", 1, &lidarPerception::ClusterCallback, this);

	_pub_clip_cloud = nh.advertise<sensor_msgs::PointCloud2>("/xh/points_clip", 1);
	_pub_in_cloud = nh.advertise<sensor_msgs::PointCloud2>("/xh/points_in", 1);
	_pub_cluster_cloud = nh.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);
	_pub_noground_cloud = nh.advertise<sensor_msgs::PointCloud2>("/xh/nopoints_ground", 1);
	_pub_ground_cloud = nh.advertise<sensor_msgs::PointCloud2>("/xh/points_ground", 1);

	_pub_clusters_message = nh.advertise<autoware_msgs::CloudClusterArray>("/detection/lidar_detector/cloud_clusters", 1);
	_pub_detected_objects = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
	_pub_cluster_visualize_markers = nh.advertise<visualization_msgs::MarkerArray>("/xh/visualize/cluster_markers", 1);

	ros::spin();
}

void transformPointCloudToFrontLeftUp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    for (auto& point : cloud->points) {
		float temp = point.y;
        point.y = point.x; 
		point.x = temp; 
    }
}

void lidarPerception::ClusterCallback(
	const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
{
	std_msgs::Header header = in_sensor_cloud->header;
	std::cout << "header : " << header.frame_id << std::endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr clip_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr noground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::fromROSMsg(*in_sensor_cloud, *in_cloud_ptr);
	// nuscenes 右前上 -> 前左上
	transformPointCloudToFrontLeftUp(in_cloud_ptr);

	std::cout << "in_cloud_ptr " << in_cloud_ptr->points.size() << std::endl;
	// 提取ROI
	int64_t tm0 = gtm();
	// preProcessing_.preprocess_(in_cloud_ptr,downsampled_cloud_ptr);
	roiClip_.GetROI(in_cloud_ptr, clip_cloud_ptr);
	std::cout << "clip_cloud_ptr " << clip_cloud_ptr->points.size() << std::endl;
	int64_t tm1 = gtm();
	ROS_INFO("ROI_Clip cost time:%ld ms", (tm1 - tm0) / 1000);

	//     // 下採樣
	voxelGridFilter_.downsample(clip_cloud_ptr, downsampled_cloud_ptr);

	// // 地面分割
	PatchWork patchWork_;
	patchWork_.estimate_ground(downsampled_cloud_ptr, ground_cloud_ptr, noground_cloud_ptr);
	int64_t tm2 = gtm();
	ROS_INFO("remove ground cost time:%ld ms", (tm2 - tm1) / 1000);
	//聚类
	pcl::PointCloud<pcl::PointXYZI>::Ptr outCloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointsVector;
	cluster_.segmentByDistance(noground_cloud_ptr, outCloudPtr, pointsVector);
	std::cout << "pointsVector : " << pointsVector.size() << std::endl;
	int64_t tm3 = gtm();
	ROS_INFO("euclidean cluster cost time:%ld ms", (tm3 - tm2) / 1000);
	ROS_INFO("total cost time:%ld ms", (tm3 - tm0) / 1000);
	//获取bounding_box信息
	autoware_msgs::CloudClusterArray inOutClusters;
	boundingBox_.getBoundingBox(header, pointsVector, inOutClusters);
	autoware_msgs::DetectedObjectArray detected_objects;
	publishDetectedObjects(inOutClusters, detected_objects);
	//可视化
	visualization_msgs::MarkerArray visualize_markers;
	vdo_.visualizeDetectedObjs(detected_objects, visualize_markers);
	_pub_cluster_visualize_markers.publish(visualize_markers);

	// 发布topic
	publishCloud(&_pub_in_cloud, in_sensor_cloud->header, in_cloud_ptr);
	publishCloud(&_pub_clip_cloud, in_sensor_cloud->header, clip_cloud_ptr);
	publishCloud(&_pub_noground_cloud, in_sensor_cloud->header, noground_cloud_ptr);
	publishCloud(&_pub_cluster_cloud, in_sensor_cloud->header, outCloudPtr);

	euclidean_time += tm3 - tm2;
	total_time += tm3 - tm0;
	counter++;
	if (counter % 100 == 0)
	{
		ROS_INFO(
			"[INFO] euclidean cluster average time per hundred times:%ld ms",
			euclidean_time / 100000);
		ROS_INFO("[INFO] total average time per hundred times:%ld ms",
				 total_time / 100000);
		euclidean_time = 0.;
		total_time = 0.;
	}
}

void lidarPerception::publishDetectedObjects(
	const autoware_msgs::CloudClusterArray &in_clusters,
	autoware_msgs::DetectedObjectArray &detected_objects)
{
	detected_objects.header = in_clusters.header;
	for (size_t i = 0; i < in_clusters.clusters.size(); i++)
	{
		autoware_msgs::DetectedObject detected_object;
		detected_object.header = in_clusters.header;
		detected_object.label = "unknown";
		detected_object.score = 1.;
		detected_object.space_frame = in_clusters.header.frame_id;
		detected_object.pose = in_clusters.clusters[i].bounding_box.pose;

		detected_object.dimensions = in_clusters.clusters[i].dimensions;
		detected_object.pointcloud = in_clusters.clusters[i].cloud;
		detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
		detected_object.valid = true;

		detected_objects.objects.push_back(detected_object);
	}

	_pub_detected_objects.publish(detected_objects);
}
