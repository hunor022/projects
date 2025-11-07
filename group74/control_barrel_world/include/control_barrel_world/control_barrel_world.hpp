#ifndef __PCL_OD_H__
#define __PCL_OD_H__

#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ControlNode : public rclcpp::Node
//class for controlling mirte, subscribing, publishing
{
public:
    //constructor
    ControlNode();
    //pedestrian subscriber callback
    void pedestrian_subscriber_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    //barrel subscriber callback
    void detection_subscriber_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);
    //publisher callback function
    void publisher_callback();
    //return the closest Detection3D object to Mirte
    vision_msgs::msg::Detection3D get_closest();

private:
    //ROS I/O
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_; //mirte/cmd_vel publisher
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_subscriber_; //detection subscriber
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr pedestrian_subscriber_; //pedestrian subscriber
    
    vision_msgs::msg::Detection3DArray::SharedPtr barrel_msg; //3d array msg for detection
    vision_msgs::msg::Detection2DArray::SharedPtr pedestrian_msg; //2d array for pedestrians
    rclcpp::TimerBase::SharedPtr timer_; //timer
    //tf2 transforms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    //parameters
    double velocity_x_driving{0.17};
    double velocity_x_steering{0.1};
    double velocity_z_steering{0.3};
    //control values
    bool pedestrian_detected = 0;
};

#endif
