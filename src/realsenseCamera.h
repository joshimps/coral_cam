#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

class RealsenseCamera: public rclcpp::Node{
    public:
        RealsenseCamera();
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cameraSubscriber_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr buttonSubscriber_;

        sensor_msgs::msg::PointCloud2 currentPointCloud_;
        sensor_msgs::msg::PointCloud2 capturedPointCloud_;
};