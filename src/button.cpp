#include "button.hpp"

///////////////////////////////////////////////////////////
// Constructors & Destructors
//////////////////////////////////////////////////////////
namespace coral_cam{

    Button::Button(const rclcpp::NodeOptions &options):Node("button_node",options){
        buttonPressedPublisher_ = this->create_publisher<std_msgs::msg::Bool>("button_pressed_topic", 10);

        gpioHandleSubscriber_ = this->create_subscription<std_msgs::msg::Int64>(
        "gpio_handle_topic", 10, std::bind(&Button::setGpioHandle, this, std::placeholders::_1));

        buttonPinSubscriber_ = this->create_subscription<std_msgs::msg::Int64>(
        "button_pin_topic", 10, std::bind(&Button::setButtonPin, this, std::placeholders::_1));

        pinConfigured_ = false;
        gpioHandle_ = -1;
        buttonPinNumber_ = -1;
        timer_ = this->create_wall_timer(1000ms, std::bind(&Button::readPin, this));
    }

    int Button::setGpioHandle(std_msgs::msg::Int64 msg){

        if(msg.data >= 0){
            RCLCPP_INFO(this->get_logger(), "GPIO HANDLE SET: %ld", msg.data);
            gpioHandle_ = msg.data;
            return 0;
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "INVALID GPIO HANDLE: %ld", msg.data);
            return 1;
        }
    }

    int Button::setButtonPin(std_msgs::msg::Int64 msg){
        if(msg.data >= 0){
            RCLCPP_INFO(this->get_logger(), "BUTTON PIN NUMBER SET: %ld", msg.data);
            buttonPinNumber_ = msg.data;
            return 0;
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "INVALID PIN NUMBER: %ld", msg.data);
            return 1;

        }
    }

    void Button::readPin(){
        int pinValue;

        //If pin has not been configured and it has valid inputs configure the pin
        if(pinConfigured_ == false && buttonPinNumber_ > -1 && gpioHandle_ > -1){
            lgGpioClaimInput(gpioHandle_,lflags_,buttonPinNumber_);
            RCLCPP_INFO(this->get_logger(), "PIN CONFIGURED");
            pinConfigured_ = true;
        }
        else if(buttonPinNumber_ < 0 ){
            RCLCPP_INFO(this->get_logger(), "PIN NUMBER HAS NOT BEEN CONFIGURED CORRECTLY: %d",buttonPinNumber_);
            return;
        }
        else if(gpioHandle_ < 0){
            RCLCPP_INFO(this->get_logger(), "GPIO HANDLE HAS NOT BEEN CONFIGURED CORRECTLY: %d",gpioHandle_);
            return;
        }

        pinValue = lgGpioRead(gpioHandle_, buttonPinNumber_);
        RCLCPP_INFO(this->get_logger(), "PIN VALUE: %d", pinValue);

        if(pinValue == 1){
            buttonPressedMessage_.data = pinValue;
            buttonPressedPublisher_->publish(buttonPressedMessage_);
        }
        else{
            return;
        }
    }

}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::Button)
