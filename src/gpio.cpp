#include "gpio.hpp"

///////////////////////////////////////////////////////////
// Constructors & Destructors
//////////////////////////////////////////////////////////
namespace coral_cam{

    Gpio::Gpio(const rclcpp::NodeOptions &options):Node("button_node",options){
        
        this->declare_parameter("gpio_number", -1);
        this->declare_parameter("button_pin", -1);

        gpioNumber_ = this->get_parameter("gpio_number").as_int();
        buttonPin_ = this->get_parameter("button_pin").as_int(); 
        gpioHandle_ = lgGpiochipOpen(gpioNumber_);

        gpioHandlePublisher_ = this->create_publisher<std_msgs::msg::Int64>("gpio_handle_topic", 10);
        buttonPinPublisher_ = this->create_publisher<std_msgs::msg::Int64>("button_pin_topic", 10);

        gpioHandleMessage_.data = gpioHandle_;
        buttonPinMessage_.data = buttonPin_;

        gpioHandlePublisher_->publish(gpioHandleMessage_);
        buttonPinPublisher_->publish(buttonPinMessage_);
    }

}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::Gpio)
