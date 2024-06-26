#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include <chrono>

namespace coral_cam
{
    class IndustrialCamera : public rclcpp::Node
    {
    public:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constructors and Destructors                                                                          //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        IndustrialCamera(const rclcpp::NodeOptions &options);

    private:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Callbacks                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        void StartWritingCapture(std_msgs::msg::Bool::SharedPtr msg);
        void GetCurrentImage();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Node, Publishers and Subscribers                                                                      //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr blue_pulse_in_progress_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr industrial_camera_capture_in_progress_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Variables                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        bool blue_pulse_in_progress_;
        bool previous_blue_pulse_in_progress_;

        int previous_number_of_captures_to_save_;
        int number_of_captures_to_save_;
    };
}