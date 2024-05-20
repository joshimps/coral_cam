#include "capture_button.hpp"

///////////////////////////////////////////////////////////
// Constructors & Destructors
//////////////////////////////////////////////////////////
namespace coral_cam
{

    CaptureButton::CaptureButton(const rclcpp::NodeOptions &options) : Node("capture_button_node", options)
    {

        this->declare_parameter("debounce_time_us", 0);
        this->declare_parameter("capture_button_pin", -1);

        gpio_handle_ = -1;
        pin_value_ = -1;
        pin_configured_ = false;

        debounce_time_us_ = this->get_parameter("debounce_time_us").as_int();
        capture_button_pin_ = this->get_parameter("capture_button_pin").as_int();

        capture_in_progress_publisher_ = this->create_publisher<std_msgs::msg::Bool>("capture_in_progress", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CaptureButton::ReadPin, this));

        gpio_handle_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "gpio_handle_topic", 10, std::bind(&CaptureButton::SetGpioHandle, this, std::placeholders::_1));
    }

    void CaptureButton::SetGpioHandle(std_msgs::msg::Int64 msg)
    {
        gpio_handle_ = msg.data;

    }

    void CaptureButton::ReadPin()
    {
        //If the GPIO Handle and PIN specified are good setup the pin for reading
        if (pin_configured_ == false && capture_button_pin_ > -1 && gpio_handle_ > -1)
        {
            lgGpioClaimInput(gpio_handle_, lflags_, capture_button_pin_);
            lgGpioSetDebounce(gpio_handle_, capture_button_pin_, debounce_time_us_);
            RCLCPP_INFO(this->get_logger(), "GPIO PIN: %d AT GPIO HANDLE: %d HAS BEEN CONFIGURED WITH DEBOUNCE TIME %d MICROSECONDS", capture_button_pin_, gpio_handle_, debounce_time_us_);
            pin_configured_ = true;
        }
        else if (capture_button_pin_ < 0)
        {
            RCLCPP_ERROR_ONCE(this->get_logger(), "CAPTURE BUTTON RECIEVED BAD PIN NUMBER: %d", capture_button_pin_);
        }
        else if (gpio_handle_ < 0)
        {
            RCLCPP_ERROR_ONCE(this->get_logger(), "CAPTURE BUTTON RECIEVED BAD GPIO HANDLE: %d", gpio_handle_);
        }

        previous_pin_value_ = pin_value_;
        pin_value_ = lgGpioRead(gpio_handle_, capture_button_pin_);

        if (pin_value_ == 1 || pin_value_ == 0)
        {
            if(pin_value_ == 0 && previous_pin_value_ == 1){
                RCLCPP_INFO(this->get_logger(), "STARTING CAPTURE");
                std_msgs::msg::Bool capture_in_progress;
                capture_in_progress.data = true;
                capture_in_progress_publisher_->publish(capture_in_progress);
            }
           
        }
        else
        {
            RCLCPP_ERROR_ONCE(this->get_logger(), "BAD PIN READ, CHECK FOR BAD WIRING");
        }
    }

}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::CaptureButton)
