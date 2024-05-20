#include "battery.hpp"

namespace coral_cam
{
    Battery::Battery(const rclcpp::NodeOptions &options) : Node("battery_node", options){
        //Every minute we will publish the battery percentage
        current_battery_publisher_ = this->create_publisher<std_msgs::msg::String>("current_battery_level", 10);
        timer_ = this->create_wall_timer(60000ms, std::bind(&Battery::ReadBatteryPercentage, this));

        //Ideally we would set up the battery reading here
    }

    void Battery::ReadBatteryPercentage(){
        //Here we would read the battery percentage from the sensor then publish it.
        //For now we will say the battery percentage is unknown

        std_msgs::msg::String message;
        battery_level_ = "UNKNOWN";
        message.data = battery_level_;
        current_battery_publisher_->publish(message);
    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::Battery)