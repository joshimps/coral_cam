//Reads pins which button is connected to and publishes a boolean signaling if button was pressed

#include "button.h"


///////////////////////////////////////////////////////////
// Constructors & Destructors
//////////////////////////////////////////////////////////

Button::Button():Node("button_node"){
    buttonPressedPublisher_ = this->create_publisher<std_msgs::msg::Bool>("button_pressed_topic", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&Button::timerCallback, this));
    //Create a timer which periodically checks the GPIO for an input signal
}

void Button::timerCallback(){
    //Periodically check GPIO for input signal

    //Publish the state of the button
    buttonPressedPublisher_->publish(buttonPressedMessage_);
    
}