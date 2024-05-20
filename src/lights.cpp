#include "lights.hpp"

namespace coral_cam
{
    Lights::Lights(const rclcpp::NodeOptions &options) : Node("lights_node", options)
    {
        this->declare_parameter("blue_flash_length", 0);

        capture_in_progress_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "capture_in_progress", 10, std::bind(&Lights::FlashBlue, this, std::placeholders::_1));

        industrial_camera_capture_in_progress_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "industrial_camera_capture_in_progress", 10, std::bind(&Lights::FlashWhite, this, std::placeholders::_1));

        blue_pulse_in_progress_publisher_ = this->create_publisher<std_msgs::msg::Bool>("blue_pulse_in_progress", 10);
        white_pulse_in_progress_publisher_ = this->create_publisher<std_msgs::msg::Bool>("white_pulse_in_progress", 10);

        industrial_camera_capture_in_progress_ = false;
    }

    void Lights::TurnOnBlueLights()
    {
        RCLCPP_INFO(this->get_logger(), "BLUE LIGHTS TURNED ON");
    }

    void Lights::TurnOffBlueLights()
    {
        RCLCPP_INFO(this->get_logger(), "BLUE LIGHTS TURNED OFF");
    }

    void Lights::TurnOnWhiteLights()
    {
        RCLCPP_INFO(this->get_logger(), "WHITE LIGHTS TURNED ON");
    }

    void Lights::TurnOffWhiteLights()
    {
        RCLCPP_INFO(this->get_logger(), "WHITE LIGHTS TURNED OFF");
    }

    void Lights::FlashBlue(std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            std_msgs::msg::Bool inProgress;
            inProgress.data = true;
            blue_pulse_in_progress_publisher_->publish(inProgress);

            blue_pulse_length_ms_ = this->get_parameter("blue_flash_length").as_int();
            RCLCPP_INFO(this->get_logger(), "BLUE LIGHT PULSE FOR: %d MILLISECONDS", blue_pulse_length_ms_);

            TurnOnBlueLights();
            rclcpp::sleep_for(std::chrono::milliseconds(blue_pulse_length_ms_));
            TurnOffBlueLights();

            inProgress.data = false;
            blue_pulse_in_progress_publisher_->publish(inProgress);
        }
    }

    void Lights::FlashWhite(std_msgs::msg::Bool::SharedPtr msg)
    {

        previous_industrial_camera_capture_in_progress_ = industrial_camera_capture_in_progress_;
        industrial_camera_capture_in_progress_ = msg->data;

        // When the industrial camera capture is in progress we should turn on the white lights
        if (previous_industrial_camera_capture_in_progress_ == true && industrial_camera_capture_in_progress_ == false)
        {
            std_msgs::msg::Bool inProgress;
            inProgress.data = true;
            white_pulse_in_progress_publisher_->publish(inProgress);

            TurnOnWhiteLights();

            inProgress.data = false;
            white_pulse_in_progress_publisher_->publish(inProgress);
        }
    }
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::Lights)