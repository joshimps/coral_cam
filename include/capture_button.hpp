#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include <lgpio.h>
#include <functional>
#include <memory>
#include <chrono>
#include <thread>

namespace coral_cam
{
    class CaptureButton : public rclcpp::Node
    {
    public:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constructors and Destructors                                                                          //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        CaptureButton(const rclcpp::NodeOptions &options);

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
        */
        void SetGpioHandle(std_msgs::msg::Int64 msg);

        /**
        Reads whether the pin specified by buttonPinNumber_ is HIGH or LOW and publishes its value to button_pressed_topic
        */
        void ReadPin();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Private Methods                                                                                       //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Node,Timers, Publishers and Subscribers                                                                      //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr capture_in_progress_publisher_;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr gpio_handle_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr button_pin_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constants                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        const int lflags_ = 0;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Variables                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        int gpio_handle_;

        int capture_button_pin_;
        int debounce_time_us_;

        int pin_value_;
        int previous_pin_value_;

        bool pin_configured_;
        bool read_error_;
    };
}