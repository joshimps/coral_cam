//After lights have been triggered takes a capture

#include "realsenseCamera.hpp"

namespace coral_cam{
    RealsenseCamera::RealsenseCamera(const rclcpp::NodeOptions & options):Node("realsense_camera_node",options){
        buttonSubscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "button_pressed_topic", 10, std::bind(&RealsenseCamera::savePointCloud, this, std::placeholders::_1));
        realsenseSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 10, std::bind(&RealsenseCamera::readCurrentPointCloud, this, std::placeholders::_1));

        this->declare_parameter("point_cloud_path", "~/Documents");
    }

    void RealsenseCamera::savePointCloud(std_msgs::msg::Bool msg){
        RCLCPP_INFO(this->get_logger(),"RECIEVED");

        if(msg.data){
            savedPointCloud_ = currentPointCloud_;
            pcl_conversions::toPCL(currentPointCloud_,savedPointCloudAsPcl_);
            writePointCloudtoFile();
        }
        else{
            RCLCPP_INFO(this->get_logger(),"ERROR, BUTTON SUBSCRIBER ACTIVATED WITH FALSE VALUE");
        }
    }

    void RealsenseCamera::readCurrentPointCloud(sensor_msgs::msg::PointCloud2 msg){
        currentPointCloud_ = msg;
    }

    void RealsenseCamera::writePointCloudtoFile(){
        Eigen::Vector4f zero = Eigen::Vector4f::Zero();
        Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();

        tm * currentLocalTime;
        time_t currentTime;

        char dateString[100];
	    char timeString[100];

        time(&currentTime);
	    currentLocalTime = localtime(&currentTime);
        
        strftime(dateString, 50, "%d%m%y", currentLocalTime);
	    strftime(timeString, 50, "%H%M%S", currentLocalTime);

        std::string path = this->get_parameter("point_cloud_path").as_string() + "/pcd_file_" + dateString + "_" +timeString + ".pcd";

        pcl::io::savePCDFile(path,savedPointCloudAsPcl_,zero,identity,true);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::RealsenseCamera)
