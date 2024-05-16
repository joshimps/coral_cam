#include "capture_button.hpp"

///////////////////////////////////////////////////////////
// Constructors & Destructors
//////////////////////////////////////////////////////////
namespace coral_cam
{

    CaptureButton::CaptureButton(const rclcpp::NodeOptions &options) : Node("capture_button_node", options)
    {

        this->declare_parameter("debounce_time_us", 0);
        this->declare_parameter("capture_button_pin", 0);

        button_pressed_publisher_ = this->create_publisher<std_msgs::msg::Bool>("pin_value", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&CaptureButton::ReadPin, this));

        gpio_handle_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "gpio_handle_topic", 10, std::bind(&CaptureButton::SetGpioHandle, this, std::placeholders::_1));

        pin_configured_ = false;
        read_error_ = false;
        gpio_handle_ = -1;

        debounce_time_us_ = this->get_parameter("debounce_time_us").as_int();
        capture_button_pin_ = this->get_parameter("capture_button_pin").as_int();
    }

    int CaptureButton::SetGpioHandle(std_msgs::msg::Int64 msg)
    {

        if (msg.data >= 0)
        {
            gpio_handle_ = msg.data;
            return 0;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "INVALID GPIO HANDLE: %ld", msg.data);
            return 1;
        }
    }

    void CaptureButton::ReadPin()
    {

        if (read_error_)
        {
            return;
        }

        int pin_value;

        if (pin_configured_ == false && capture_button_pin_ > -1 && gpio_handle_ > -1)
        {
            lgGpioClaimInput(gpio_handle_, lflags_, capture_button_pin_);
            lgGpioSetDebounce(gpio_handle_, capture_button_pin_, debounce_time_us_);
            RCLCPP_INFO(this->get_logger(), "GPIO PIN: %d AT GPIO HANDLE: %d HAS BEEN CONFIGURED WITH DEBOUNCE TIME %d MICROSECONDS", capture_button_pin_, gpio_handle_, debounce_time_us_);
            pin_configured_ = true;
        }
        else if (capture_button_pin_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "CAPTURE BUTTON RECIEVED BAD PIN NUMBER: %d", capture_button_pin_);
            read_error_ = true;
            return;
        }
        else if (gpio_handle_ < 0)
        {
            read_error_ = true;
            RCLCPP_ERROR(this->get_logger(), "CAPTURE BUTTON RECIEVED BAD GPIO HANDLE: %d", gpio_handle_);
            return;
        }

        pin_value = lgGpioRead(gpio_handle_, capture_button_pin_);

        if (pin_value == 1 || pin_value == 0)
        {
            std_msgs::msg::Bool button_pressed_message;
            button_pressed_message.data = pin_value;
            button_pressed_publisher_->publish(button_pressed_message);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "BAD PIN READ, CHECK FOR BAD WIRING");
            return;
        }
    }

}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::CaptureButton)
