//Reads pins which button is connected to and publishes a boolean signaling if button was pressed

#include "button.h"


///////////////////////////////////////////////////////////
// Constructors & Destructors
//////////////////////////////////////////////////////////

Button::Button(int buttonPinNumber):Node("button_node"){
    buttonPinNumber_ = buttonPinNumber;
    gpioHandle_ = lgGpiochipOpen(0);
    lgGpioClaimInput(gpioHandle_,lflags_,buttonPinNumber_);

    buttonPressedPublisher_ = this->create_publisher<std_msgs::msg::Bool>("button_pressed_topic", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&Button::timerCallback, this));
    //Create a timer which periodically checks the GPIO for an input signal
}

void Button::timerCallback(){
    //Periodically check GPIO for input signal
    int pinValue;

    pinValue = lgGpioRead(gpioHandle_, buttonPinNumber_);
    if (pinValue == 1)
    {
       RCLCPP_INFO(get_logger(),"BUTTON PRESSED");
    }

    buttonPressedMessage_.data = pinValue;

    //Publish the state of the button
    buttonPressedPublisher_->publish(buttonPressedMessage_);

}