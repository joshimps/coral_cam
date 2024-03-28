#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

class RealsenseCamera: public rclcpp::Node{
    public:

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constructors and Destructors                                                                          //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        RealsenseCamera();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Public Methods                                                                                        //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
        Saves the current point cloud seen by the Intel Realsense camera
        @return capture successful
        */
        int capturePointCloud();


        /**
        Returns the current point cloud save in currentPointCloud_;
        @return capture successful
        */
        sensor_msgs::msg::PointCloud2 getCurrentPointCloud();

    private:

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Callbacks                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Private Methods                                                                                       //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Node, Publishers and Subscribers                                                                      //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr realsenseSubscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2 >::SharedPtr capturedPointCloudPublisher_;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constants                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Variables                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        sensor_msgs::msg::PointCloud2 currentPointCloud_;
        sensor_msgs::msg::PointCloud2 capturedPointCloud_;
};