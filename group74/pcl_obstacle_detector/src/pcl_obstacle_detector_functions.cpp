#include "pcl_obstacle_detector/pcl_obstacle_detector.hpp"

//constructor definition, subscriber, publisher
ObstacleDetector::ObstacleDetector() : rclcpp::Node("pcl_obstacle_detector")
{
  //publish
  publisher = this->create_publisher<vision_msgs::msg::Detection3DArray>("detections", 10);

	auto sub_cb = [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
	{
		this->pc_msg = msg;
    //initialization of cluster_msg
    this->cluster_msg = std::make_shared<vision_msgs::msg::Detection3DArray>();
    //copy header to main msg
    this->cluster_msg->header = msg->header;
    //call conversion
    this->convert_pc();
    //call filter
    this->filter_pc();
    //call ground filter
    this->ground_filter_pc();
    //call the segmentation function
    this->segmentation_pc();
    //call the cluster converter
    this->convert_cluster();
    //publish the msg at the end of sub cycle
    this->publisher->publish(*this->cluster_msg);
	};

	subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("mirte/camera_depth/points", 1, sub_cb);
}

//convert PointCloud2 to PointCloud<PointXYZ>
void ObstacleDetector::convert_pc()
{
  //initialize
  this->cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  
  pcl::fromROSMsg(*this->pc_msg, *this->cloud);
}

//filter out Nan and too far away points
void ObstacleDetector::filter_pc()
{
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (this->cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setNegative (true);
  pass.filter (*this->cloud);
}

//filter out the ground plane
void ObstacleDetector::ground_filter_pc()
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01); //1 cm as described in the manual
  seg.setInputCloud (this->cloud);
  seg.segment (*inliers, *coefficients);

  // Create the filtering object
  extract.setInputCloud (this->cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_filtered);
  this->cloud = cloud_filtered;
}

//segment the cones/barrels into clusters
void ObstacleDetector::segmentation_pc()
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (this->cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.01); // 1 cm, as described in the manual
  ec.setMinClusterSize (5); //as described in the manual
  ec.setMaxClusterSize (25000); //as described in the manual
  ec.setSearchMethod (tree);
  ec.setInputCloud (this->cloud);
  ec.extract (cluster_indices);

  this->cloud_cluster_array.clear();
  int j = 0;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices)
    {
      cloud_cluster->push_back((*this->cloud)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    this->cloud_cluster_array.push_back(cloud_cluster);

    j++;
  }
}

//take the PointCloud<PointXYZ> vector and make the boundingbox, refresh the message
void ObstacleDetector::convert_cluster()
{
  vision_msgs::msg::Detection3D detection;
  Eigen::Vector4f centroid;
  pcl::PointXYZ min_point, max_point;
  for(int i = 0; i < (int)this->cloud_cluster_array.size(); i++)
  {
    pcl::compute3DCentroid(*this->cloud_cluster_array.at(i), centroid);
    detection.bbox.center.position.x = centroid[0];
    detection.bbox.center.position.y = centroid[1];
    detection.bbox.center.position.z = centroid[2];
    
    pcl::getMinMax3D(*this->cloud_cluster_array.at(i), min_point, max_point);
    detection.bbox.size.x = std::abs(max_point.x - min_point.x);
    detection.bbox.size.y = std::abs(max_point.y - min_point.y);
    detection.bbox.size.z = std::abs(max_point.z - min_point.z);

    detection.header = this->cluster_msg->header;
    this->cluster_msg->detections.push_back(detection);
  }
}