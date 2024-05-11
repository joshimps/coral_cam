#include "gpio.hpp"

///////////////////////////////////////////////////////////
// Constructors & Destructors
//////////////////////////////////////////////////////////
namespace coral_cam{

    Gpio::Gpio(const rclcpp::NodeOptions &options):Node("gpio_node",options){
        
        this->declare_parameter("gpio_number", 0);

        gpio_number_ = this->get_parameter("gpio_number").as_int();
        gpio_handle_ = lgGpiochipOpen(gpio_number_);

        gpio_handle_publisher_ = this->create_publisher<std_msgs::msg::Int64>("gpio_handle_topic", 10);
        
        std_msgs::msg::Int64 gpio_handle_message;
        gpio_handle_message.data = gpio_handle_;
        gpio_handle_publisher_->publish(gpio_handle_message);
    }

}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::Gpio)
