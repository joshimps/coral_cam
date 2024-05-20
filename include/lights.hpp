#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/bool.hpp"

#include <chrono>

namespace coral_cam
{
    class Lights : public rclcpp::Node
    {
    public:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constructors and Destructors                                                                          //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        Lights(const rclcpp::NodeOptions &options);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Public Methods                                                                                        //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        void TurnOnBlueLights();
        void TurnOffBlueLights();

        void TurnOnWhiteLights();
        void TurnOffWhiteLights();

    private:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Callbacks                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        void FlashBlue(std_msgs::msg::Bool::SharedPtr msg);
        void FlashWhite(std_msgs::msg::Bool::SharedPtr msg);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Node, Publishers and Subscribers                                                                      //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr capture_in_progress_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr industrial_camera_capture_in_progress_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr blue_pulse_in_progress_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr white_pulse_in_progress_publisher_;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Variables                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        int blue_pulse_length_ms_;

        bool previous_industrial_camera_capture_in_progress_;
        bool industrial_camera_capture_in_progress_;
    };
}