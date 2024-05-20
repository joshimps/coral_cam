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

        void StartWritingPointCloud(std_msgs::msg::Bool::SharedPtr msg);

        void GetCurrentPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Node, Publishers and Subscribers                                                                      //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr real_sense_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr industrial_camera_capture_in_progress_subscriber_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr real_sense_capture_in_progress_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr capture_in_progress_publisher_;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Variables                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        std::string path_;
        int number_of_files_;

        int previous_number_of_captures_to_save_;
        int number_of_captures_to_save_;

        sensor_msgs::msg::PointCloud2 current_point_cloud_;
        pcl::PCLPointCloud2 saved_point_cloud_as_pcl_;

        bool previous_industrial_camera_capture_in_progress_;
        bool industrial_camera_capture_in_progress_;
    };
}