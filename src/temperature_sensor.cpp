#include "temperature_sensor.hpp"

namespace coral_cam
{
    TemperatureSensor::TemperatureSensor(const rclcpp::NodeOptions &options) : Node("temperature_node", options)
    {
        // Every 5s we will publish the current temperature
        this->declare_parameter("cutoff_temperature", 90);
        current_temperature_publisher_ = this->create_publisher<std_msgs::msg::String>("current_temperature", 10);
        timer_ = this->create_wall_timer(5000ms, std::bind(&TemperatureSensor::ReadCurrentTemperature, this));
        current_temperature_ = -274;
        // Ideally we would set up the battery reading here
    }

    void TemperatureSensor::ReadCurrentTemperature()
    {
        // Here we would read the battery percentage from the sensor then publish it.
        // For now we will say the battery percentage is unknown
        cutoff_temperature_ = this->get_parameter("cutoff_temperature").as_int();
        if (current_temperature_ >= cutoff_temperature_)
        {
            RCLCPP_ERROR(this->get_logger(), "OVERHEAT, SHUTTING DOWN AS %d EXCEEDS THE CUTOFF TEMPERATURE OF %d", current_temperature_, cutoff_temperature_);
            throw std::out_of_range("OVERHEAT, SHUTTING DOWN");
        }
        else if (current_temperature_ <= -274)
        {
            current_temperature_string_ = "UNKNOWN";
        }
        else
        {
            current_temperature_string_ = std::to_string(current_temperature_);
        }

        std_msgs::msg::String message;
        message.data = current_temperature_string_;
        current_temperature_publisher_->publish(message);
    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::TemperatureSensor)