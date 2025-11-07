#ifndef __PCL_OD_H__
#define __PCL_OD_H__

#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

class ObstacleDetector : public rclcpp::Node
//class for detecting objects, subscribing, publishing
{
public:
	//constructor function
	ObstacleDetector();
    //converter function from PointCloud2 to PoinCloud<PointXYZ>
	void convert_pc();
	//filtering function (NaN, far points)
	void filter_pc();
	//ground filtering function
	void ground_filter_pc();
	//segmentation function
	void segmentation_pc();
	//convert PointCloud<PointXYZ> arrays to Detection3DArray
	void convert_cluster();
private:
	sensor_msgs::msg::PointCloud2::SharedPtr pc_msg; //PointCloud2 incoming message
	vision_msgs::msg::Detection3DArray::SharedPtr cluster_msg; //output cluster msg
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription; //subscriber function
	rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher; //publisher function
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; //PointCloud<PointXYZ> that gets modified
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cluster_array; //clustered PointCloud<PointXYZ> for detections
};

#endif