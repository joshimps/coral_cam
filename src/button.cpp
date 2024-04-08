#include "button.hpp"

///////////////////////////////////////////////////////////
// Constructors & Destructors
//////////////////////////////////////////////////////////
namespace coral_cam{
    Button::Button(const rclcpp::NodeOptions &options):Node("button_node",options){
        this->declare_parameter("button_pin_number", -1);
        buttonPinNumber_ = this->get_parameter("button_pin_number").as_int();
        buttonPressedPublisher_ = this->create_publisher<std_msgs::msg::Bool>("button_pressed_topic", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&Button::timerCallback, this));
    }

    void Button::timerCallback(){
        //Periodically check GPIO for input signal
        int pinValue;
        
        this->get_parameter_or("gpio_handle", gpioHandle_, -1);
        
        if(gpioHandle_ >= 0){
            RCLCPP_INFO(this->get_logger(), "Successfully retrieved GPIO Handle %d", gpioHandle_);
            pinValue = lgGpioRead(gpioHandle_, buttonPinNumber_);

            if (pinValue == 1)
            {
            RCLCPP_INFO(get_logger(),"BUTTON PRESSED");
            }

            buttonPressedMessage_.data = pinValue;

            //Publish the state of the button
            buttonPressedPublisher_->publish(buttonPressedMessage_);
        }
        else{
            RCLCPP_INFO(this->get_logger(), "GPIO HANDLE NOT VALID");
        }
       

    }

}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::Button)
