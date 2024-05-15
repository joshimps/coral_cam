#include "temperature_sensor.hpp"

namespace coral_cam
{
    TemperatureSensor::TemperatureSensor(const rclcpp::NodeOptions &options) : Node("battery_node", options){
        //Every 5s we will publish the current temperature
        timer_ = this->create_wall_timer(5000ms, std::bind(&TemperatureSensor::ReadCurrentTemperature, this));

        //Ideally we would set up the battery reading here
    }

    void TemperatureSensor::ReadCurrentTemperature(){
        //Here we would read the battery percentage from the sensor then publish it.
        //For now we will say the battery percentage is unknown

        current_temperature_ = "UNKNOWN";
    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::TemperatureSensor)