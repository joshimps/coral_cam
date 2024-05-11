#include "realsenseCamera.hpp"
#include "helpers.hpp"

namespace coral_cam{
    RealsenseCamera::RealsenseCamera(const rclcpp::NodeOptions & options):Node("realsense_camera_node",options){

        this->declare_parameter("point_cloud_path", "~/Documents");
        this->declare_parameter("number_of_captures", 10);

        buttonSubscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "pin_value", 10, std::bind(&RealsenseCamera::savePointCloud, this, std::placeholders::_1));

        realsenseSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/real_sense/depth/color/points", 10, std::bind(&RealsenseCamera::readCurrentPointCloud, this, std::placeholders::_1));

        currentButtonValue_ = 0;
        previousButtonValue_ = 0;

        path_ = this->get_parameter("point_cloud_path").as_string() + "/pcd_file_" +  getCurrentTime() + ".pcd";
        numberOfFiles_ = 0;
        numberOfFilesToSave_ = 0;
    }

    void RealsenseCamera::savePointCloud(std_msgs::msg::Bool msg){

        previousButtonValue_ = currentButtonValue_;
        currentButtonValue_ = msg.data;

        //If the capture button has been released (i.e falling edge 1 -> 0) we save the number of point clouds specified in the number_of_captures parameter tp pcd files

        if(currentButtonValue_ == 0 && previousButtonValue_ == 1){

            numberOfFilesToSave_ = this->get_parameter("number_of_captures").as_int();
        }
    }

    void RealsenseCamera::readCurrentPointCloud(sensor_msgs::msg::PointCloud2 msg){
        currentPointCloud_ = msg;
        pcl_conversions::toPCL(currentPointCloud_,savedPointCloudAsPcl_);

        if(numberOfFilesToSave_ > 0){
            writePointCloudtoFile(savedPointCloudAsPcl_);
            numberOfFilesToSave_--;
        }
    }

    void RealsenseCamera::writePointCloudtoFile(pcl::PCLPointCloud2 pointcloud){
        Eigen::Vector4f zero = Eigen::Vector4f::Zero();
        Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
        const char* pathChar = path_.c_str(); 

        if(fileExists(pathChar)){
            RCLCPP_INFO(this->get_logger(),"FILE EXISTS");
            numberOfFiles_++;
            path_ = this->get_parameter("point_cloud_path").as_string() + "/pcd_file_" +  getCurrentTime() + "_" + std::to_string(numberOfFiles_) + ".pcd";
            RCLCPP_INFO(this->get_logger(), "NEW PATH: %s ",path_.c_str());
        }
        else{
            numberOfFiles_ = 0;
        }

        pcl::io::savePCDFile(path_,pointcloud,zero,identity,true);
    }
    
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::RealsenseCamera)
