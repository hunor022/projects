#include "control_barrel_world/control_barrel_world.hpp"

using namespace std::chrono_literals;

ControlNode::ControlNode() : rclcpp::Node("control_barrel_world")
{
    //parameters
    velocity_x_driving = this->declare_parameter<double>("velocity_x_driving", 0.17);
    velocity_x_steering = this->declare_parameter<double>("volicity_x_steering", 0.1);
    velocity_z_steering = this->declare_parameter<double>("velocity_z_steering", 0.3);

    //initialize msgs
    this->barrel_msg = std::make_shared<vision_msgs::msg::Detection3DArray>();
    this->pedestrian_msg = std::make_shared<vision_msgs::msg::Detection2DArray>();

    //transformations
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    //subscribers
    pedestrian_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2DArray>("pedestrians", 1, std::bind(&ControlNode::pedestrian_subscriber_callback, this, std::placeholders::_1));
    detection_subscriber_ = this->create_subscription<vision_msgs::msg::Detection3DArray>("detections", 1, std::bind(&ControlNode::detection_subscriber_callback, this, std::placeholders::_1));

    //publisher
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("mirte/cmd_vel", 10);
    //subscriber topics are roughly ~4-5Hz, so 25ms should be in line with them
    timer_ = this->create_wall_timer(25ms, std::bind(&ControlNode::publisher_callback, this));
}

void ControlNode::pedestrian_subscriber_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    this->pedestrian_msg = msg;
    if(!this->pedestrian_msg->detections.empty())
    {
        int bbox_size = this->pedestrian_msg->detections[0].bbox.size_x * this->pedestrian_msg->detections[0].bbox.size_y;
        if(bbox_size > 2500)
        {
            this->pedestrian_detected = 1;
        }
        else
        {
            this->pedestrian_detected = 0;
        }
    }
    else
    {
        this->pedestrian_detected = 0;
    }
}

void ControlNode::detection_subscriber_callback(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
    this->barrel_msg = msg;
    //transform
    vision_msgs::msg::Detection3DArray::SharedPtr output_msg = std::make_shared<vision_msgs::msg::Detection3DArray>();;
    output_msg->header = this->barrel_msg->header;
    output_msg->header.frame_id = "base_link";

    for(const auto& detection : this->barrel_msg->detections)
    {
        vision_msgs::msg::Detection3D output_detection;

        geometry_msgs::msg::PointStamped point_in, point_out;
        point_in.header = detection.header;
        point_in.point = detection.bbox.center.position;

        tf_buffer_->transform(point_in, point_out, "base_link", tf2::durationFromSec(0.1));
        output_detection.header = point_out.header;
        output_detection.bbox.center.position = point_out.point;

        //if the output_detetion x > 0, and the distance from (0, 0, 0) is less than 0.7
        if(output_detection.bbox.center.position.x > 0 && 0.7 > std::sqrt(std::pow(output_detection.bbox.center.position.x, 2) 
        + std::pow(output_detection.bbox.center.position.y, 2) + std::pow(output_detection.bbox.center.position.z, 2)))
        {
            //add the detection to the msg
            output_msg->detections.push_back(output_detection);
        }
    }

    this->barrel_msg = output_msg;
}

void ControlNode::publisher_callback()
{
    geometry_msgs::msg::Twist cmd;
    
    if(this->pedestrian_detected)
    {
        //stop - pedestrian detected
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }
    else
    {
        if(this->barrel_msg->detections.empty())
        {
            //drive forward
            cmd.linear.x = velocity_x_driving;
            cmd.angular.z = 0.0;
        }
        else
        {
            vision_msgs::msg::Detection3D closest;
            closest = this->get_closest();
            if(closest.bbox.center.position.y < 0)
            {
                //steer and drive right
                cmd.linear.x = velocity_x_steering;
                cmd.angular.z = velocity_z_steering;
            }
            else if(closest.bbox.center.position.y > 0)
            {
                //steer and drive left
                cmd.linear.x = velocity_x_steering;
                cmd.angular.z = -velocity_z_steering;
            }
            else //it is in the middle, we dont know what to do
            {
                //stop
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            }
        }
    }

    this->velocity_publisher_->publish(cmd);
}

//return the closest Detection3D object to Mirte
vision_msgs::msg::Detection3D ControlNode::get_closest()
{
    vision_msgs::msg::Detection3D closest = this->barrel_msg->detections[0];
    for(int i = 0; i < (int)this->barrel_msg->detections.size(); i++)
    {
        for(const auto& detection : this->barrel_msg->detections)
        {
            //if the current detection is closer to the (0, 0, 0) position than the closest
            if(std::sqrt(std::pow(detection.bbox.center.position.x, 2) + std::pow(detection.bbox.center.position.y, 2)
            + std::pow(detection.bbox.center.position.z, 2)) < std::sqrt(std::pow(closest.bbox.center.position.x, 2)
            + std::pow(detection.bbox.center.position.y, 2) + std::pow(closest.bbox.center.position.z, 2)))
            {
                //make the detection the closest
                closest = detection;
            }
        }
    }
    return closest;
}
