#include "real_sense_camera.hpp"
#include "helpers.hpp"

namespace coral_cam
{
    RealSenseCamera::RealSenseCamera(const rclcpp::NodeOptions &options) : Node("real_sense_camera_node", options)
    {

        this->declare_parameter("point_cloud_path", "~/Documents");
        this->declare_parameter("number_of_captures", 10);

        industrial_camera_capture_in_progress_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "industrial_camera_capture_in_progress", 10, std::bind(&RealSenseCamera::StartWritingPointCloud, this, std::placeholders::_1));

        real_sense_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/real_sense/depth/color/points", 10, std::bind(&RealSenseCamera::GetCurrentPointCloud, this, std::placeholders::_1));

        real_sense_capture_in_progress_publisher_ = this->create_publisher<std_msgs::msg::Bool>("real_sense_capture_in_progress", 10);

        capture_in_progress_publisher_ = this->create_publisher<std_msgs::msg::Bool>("capture_in_progress", 10);

        path_ = this->get_parameter("point_cloud_path").as_string() + "/pcd_file_" + GetCurrentTime() + ".pcd";
        number_of_files_ = 0;
        number_of_captures_to_save_ = 0;
        industrial_camera_capture_in_progress_ = false;
    }

    void RealSenseCamera::StartWritingPointCloud(std_msgs::msg::Bool::SharedPtr msg)
    {
        previous_industrial_camera_capture_in_progress_ = industrial_camera_capture_in_progress_;
        industrial_camera_capture_in_progress_ = msg->data;

        if (previous_industrial_camera_capture_in_progress_ == true && industrial_camera_capture_in_progress_ == false)
        {
            std_msgs::msg::Bool inProgress;
            inProgress.data = true;
            real_sense_capture_in_progress_publisher_->publish(inProgress);
            RCLCPP_INFO(this->get_logger(), "STARTING REAL SENSE CAPTURE");
            number_of_captures_to_save_ = this->get_parameter("number_of_captures").as_int();
        }
    }

    void RealSenseCamera::GetCurrentPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        current_point_cloud_ = *msg;
        pcl_conversions::toPCL(current_point_cloud_, saved_point_cloud_as_pcl_);

        if (number_of_captures_to_save_ > 0)
        {
            WritePointCloudtoFile(saved_point_cloud_as_pcl_);
            previous_number_of_captures_to_save_ = number_of_captures_to_save_;
            number_of_captures_to_save_--;
        }
        else if (previous_number_of_captures_to_save_ == 1)
        {
            previous_number_of_captures_to_save_ = 0;
            std_msgs::msg::Bool inProgress;
            inProgress.data = false;
            RCLCPP_INFO(this->get_logger(), "REAL SENSE CAMERA CAPTURE COMPLETE");
            real_sense_capture_in_progress_publisher_->publish(inProgress);
            RCLCPP_INFO(this->get_logger(), "FULL CAPTURE COMPLETED SUCCESSFULLY");
            capture_in_progress_publisher_->publish(inProgress);
        }
    }

    void RealSenseCamera::WritePointCloudtoFile(pcl::PCLPointCloud2 point_cloud)
    {
        Eigen::Vector4f zero = Eigen::Vector4f::Zero();
        Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
        path_ = this->get_parameter("point_cloud_path").as_string() + "/pcd_file_" + GetCurrentTime() + ".pcd";
        const char *path_char = path_.c_str();

        if (FileExists(path_char))
        {
            number_of_files_++;
            path_ = this->get_parameter("point_cloud_path").as_string() + "/pcd_file_" + GetCurrentTime() + "_" + std::to_string(number_of_files_) + ".pcd";
        }
        else
        {
            number_of_files_ = 0;
        }

        pcl::io::savePCDFile(path_, point_cloud, zero, identity, true);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::RealSenseCamera)
