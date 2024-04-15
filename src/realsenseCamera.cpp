//After lights have been triggered takes a capture

#include "realsenseCamera.hpp"

namespace coral_cam{
    RealsenseCamera::RealsenseCamera(const rclcpp::NodeOptions & options):Node("realsense_camera_node",options){
        buttonSubscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "button_pressed_topic", 10, std::bind(&RealsenseCamera::capturePointCloud, this, std::placeholders::_1));

    }

    void RealsenseCamera::capturePointCloud(std_msgs::msg::Bool msg){
        RCLCPP_INFO(this->get_logger(),"RECIEVED");
        capturedPointCloud_ = currentPointCloud_;
    }

    sensor_msgs::msg::PointCloud2 RealsenseCamera::getCurrentPointCloud(){
        return currentPointCloud_;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::RealsenseCamera)
