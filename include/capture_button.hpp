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

    private:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Callbacks                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
        Reads gpio_handle_topic and sets gpio_handle_
        */
        void SetGpioHandle(std_msgs::msg::Int64 msg);

        /**
        Reads whether the pin specified by capture_button_pin_ is HIGH or LOW and publishes true to /coral_cam/capture_in_progress
        if it reads a falling edge
        */
        void ReadPin();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Node,Timers, Publishers and Subscribers                                                               //
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