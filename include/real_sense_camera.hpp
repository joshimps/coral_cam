#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>


namespace coral_cam
{
    class RealSenseCamera : public rclcpp::Node
    {
    public:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constructors and Destructors                                                                          //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        RealSenseCamera(const rclcpp::NodeOptions &options);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Public Methods                                                                                        //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
        Writes the supplied pcl point cloud to a .pcd file
        */
        void WritePointCloudtoFile(pcl::PCLPointCloud2 pointcloud);


    private:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Callbacks                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        /**
        Saves the current point cloud seen by the Intel Realsense camera if the capture button is pressed
        */
        void SavePointCloud(std_msgs::msg::Bool::SharedPtr msg);

        /**
        Returns the current point cloud save in currentPointCloud_;
        */
        void ReadCurrentPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);


        void CalculateCurrentCentreDepth(sensor_msgs::msg::Image::SharedPtr msg);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Private Methods                                                                                       //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Node, Publishers and Subscribers                                                                      //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr real_sense_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr capture_button_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subscriber_;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constants                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Variables                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        std::string path_;
        int number_of_files_;
        int number_of_files_to_save_;

        sensor_msgs::msg::PointCloud2 current_point_cloud_;
        pcl::PCLPointCloud2 saved_point_cloud_as_pcl_;

        int pixels_[2];

        int current_capture_button_value_;
        int previous_capture_button_value_;
    };
}