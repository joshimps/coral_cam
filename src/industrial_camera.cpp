#include "industrial_camera.hpp"

namespace coral_cam
{
    IndustrialCamera::IndustrialCamera(const rclcpp::NodeOptions &options) : Node("industrial_camera_node", options)
    {
        this->declare_parameter("number_of_captures", 10);

        blue_pulse_in_progress_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "blue_pulse_in_progress", 10, std::bind(&IndustrialCamera::StartWritingCapture, this, std::placeholders::_1));

        industrial_camera_capture_in_progress_publisher_ = this->create_publisher<std_msgs::msg::Bool>("industrial_camera_capture_in_progress", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&IndustrialCamera::GetCurrentImage, this));

        blue_pulse_in_progress_ = false;
    }

    void IndustrialCamera::StartWritingCapture(std_msgs::msg::Bool::SharedPtr msg)
    {

        previous_blue_pulse_in_progress_ = blue_pulse_in_progress_;
        blue_pulse_in_progress_ = msg->data;

        // When the blue pulse is done start the capture
        if (previous_blue_pulse_in_progress_ == true && blue_pulse_in_progress_ == false)
        {
            RCLCPP_INFO(this->get_logger(), "STARTING INDUSTRIAL CAMERA CAPTURE");
            std_msgs::msg::Bool inProgress;
            inProgress.data = true;
            industrial_camera_capture_in_progress_publisher_->publish(inProgress);
            number_of_captures_to_save_ = this->get_parameter("number_of_captures").as_int();
        }
    }

    void IndustrialCamera::GetCurrentImage()
    {

        // Get the image from the industrial camera

        // If we have captures left to save, save them to file
        if (number_of_captures_to_save_ > 0)
        {
            previous_number_of_captures_to_save_ = number_of_captures_to_save_;
            number_of_captures_to_save_--;
        }
        // If we saved our last capture set the capture to no longer in progress
        else if (previous_number_of_captures_to_save_ == 1)
        {
            previous_number_of_captures_to_save_ = 0;
            std_msgs::msg::Bool inProgress;
            inProgress.data = false;
            RCLCPP_INFO(this->get_logger(), "INDUSTRIAL CAMERA CAPTURE COMPLETE");
            industrial_camera_capture_in_progress_publisher_->publish(inProgress);
        }
    }
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::IndustrialCamera)