#include "temperature_sensor.hpp"

namespace coral_cam
{
    TemperatureSensor::TemperatureSensor(const rclcpp::NodeOptions &options) : Node("temperature_node", options){
        //Every 5s we will publish the current temperature
        current_temperature_publisher_ = this->create_publisher<std_msgs::msg::String>("current_temperature", 10);
        timer_ = this->create_wall_timer(5000ms, std::bind(&TemperatureSensor::ReadCurrentTemperature, this));

        //Ideally we would set up the battery reading here
    }

    void TemperatureSensor::ReadCurrentTemperature(){
        //Here we would read the battery percentage from the sensor then publish it.
        //For now we will say the battery percentage is unknown

        std_msgs::msg::String message;
        current_temperature_ = "UNKNOWN";
        message.data = current_temperature_;
        current_temperature_publisher_->publish(message);
    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::TemperatureSensor)