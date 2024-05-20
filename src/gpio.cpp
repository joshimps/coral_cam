#include "gpio.hpp"

///////////////////////////////////////////////////////////
// Constructors & Destructors
//////////////////////////////////////////////////////////
namespace coral_cam
{

    Gpio::Gpio(const rclcpp::NodeOptions &options) : Node("gpio_node", options)
    {

        this->declare_parameter("gpio_number", -1);

        gpio_number_ = this->get_parameter("gpio_number").as_int();
        gpio_handle_ = lgGpiochipOpen(gpio_number_);

        if (gpio_handle_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "INVALID GPIO HANDLE CREATED: %d", gpio_handle_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "SUCESSFULLY CREATED GPIO HANDLE: %d", gpio_handle_);
        }

        gpio_handle_publisher_ = this->create_publisher<std_msgs::msg::Int64>("gpio_handle_topic", 1);

        if (gpio_handle_publisher_->get_intra_process_subscription_count() == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "GPIO HANDLE SUB COUNT IS 0, GPIO CONNECTIONS WILL NOT WORK: %ld", gpio_handle_publisher_->get_intra_process_subscription_count());
        }

        std_msgs::msg::Int64 gpio_handle_message;
        gpio_handle_message.data = gpio_handle_;
        gpio_handle_publisher_->publish(gpio_handle_message);
    }

}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::Gpio)
