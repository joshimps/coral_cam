#include "real_sense_camera.hpp"
#include "helpers.hpp"

namespace coral_cam{
    RealSenseCamera::RealSenseCamera(const rclcpp::NodeOptions & options):Node("realsense_camera_node",options){

        this->declare_parameter("point_cloud_path", "~/Documents");
        this->declare_parameter("number_of_captures", 10);

        capture_button_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "pin_value", 10, std::bind(&RealSenseCamera::SavePointCloud, this, std::placeholders::_1));

        real_sense_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/real_sense/depth/color/points", 10, std::bind(&RealSenseCamera::ReadCurrentPointCloud, this, std::placeholders::_1));

        current_capture_button_value_ = 0;
        previous_capture_button_value_ = 0;

        path_ = this->get_parameter("point_cloud_path").as_string() + "/pcd_file_" +  GetCurrentTime() + ".pcd";
        number_of_files_ = 0;
        number_of_files_to_save_ = 0;
    }

    void RealSenseCamera::SavePointCloud(std_msgs::msg::Bool msg){

        previous_capture_button_value_ = current_capture_button_value_;
        current_capture_button_value_ = msg.data;

        //If the capture button has been released (i.e falling edge 1 -> 0) we save the number of point clouds specified in the number_of_captures parameter tp pcd files

        if(current_capture_button_value_ == 0 && previous_capture_button_value_ == 1){

            number_of_files_to_save_ = this->get_parameter("number_of_captures").as_int();
        }
    }

    void RealSenseCamera::ReadCurrentPointCloud(sensor_msgs::msg::PointCloud2 msg){
        current_point_cloud_ = msg;
        pcl_conversions::toPCL(current_point_cloud_,saved_point_cloud_as_pcl_);

        if(number_of_files_to_save_ > 0){
            WritePointCloudtoFile(saved_point_cloud_as_pcl_);
            number_of_files_to_save_--;
        }
    }

    void RealSenseCamera::WritePointCloudtoFile(pcl::PCLPointCloud2 point_cloud){
        Eigen::Vector4f zero = Eigen::Vector4f::Zero();
        Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
        const char* path_char = path_.c_str(); 

        if(FileExists(path_char)){
            RCLCPP_INFO(this->get_logger(),"FILE EXISTS");
            number_of_files_++;
            path_ = this->get_parameter("point_cloud_path").as_string() + "/pcd_file_" +  GetCurrentTime() + "_" + std::to_string(number_of_files_) + ".pcd";
            RCLCPP_INFO(this->get_logger(), "NEW PATH: %s ",path_.c_str());
        }
        else{
            number_of_files_ = 0;
        }

        pcl::io::savePCDFile(path_,point_cloud,zero,identity,true);
    }
    
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::RealSenseCamera)
