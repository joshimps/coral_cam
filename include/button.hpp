#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include <lgpio.h>
#include <functional>
#include <memory>
#include <chrono> 
#include <thread>

using namespace std::chrono_literals;

namespace coral_cam{
    class Button : public rclcpp::Node{
        public:

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Constructors and Destructors                                                                          //
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
            Button(const rclcpp::NodeOptions &options);

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Public Methods                                                                                        //
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
       
        private:
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Callbacks                                                                                             //
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
           
            /**
            When the gpio handle is published to gpio_handle_topic this callback checks if the handle is valid and
            saves it to gpioHandle_ if valid
            @return int, 0 if valid GPIO handle set, 1 if invalid GPIO handle retrieved
            */
            int setGpioHandle(std_msgs::msg::Int64 msg);

            /**
            When the button pin is published to button_pin_topic this callback checks if the pin is valid and
            saves it to buttonPinNumber_ if valid
            @return int, 0 if valid button pin set, 1 if invalid button pin retrieved
            */
            int setButtonPin(std_msgs::msg::Int64 msg);

            /**
            Reads whether the pin specified by buttonPinNumber_ is HIGH or LOW and publishes its value to button_pressed_topic
            */
            void readPin();

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Private Methods                                                                                       //
            //////////////////////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Node,Timers, Publishers and Subscribers                                                                      //
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr buttonPressedPublisher_;
            rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr gpioHandleSubscriber_;
            rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr buttonPinSubscriber_;
            rclcpp::TimerBase::SharedPtr timer_;

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Constants                                                                                             //
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
            const int lflags_ = 0;

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Variables                                                                                             //
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
            int gpioHandle_;
            int buttonPinNumber_;
            bool pinConfigured_;
            bool readError_;
            std_msgs::msg::Bool buttonPressedMessage_;

    };  
}