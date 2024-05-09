//After lights have been triggered takes a capture

#include "realsenseCamera.hpp"

namespace coral_cam{
    RealsenseCamera::RealsenseCamera(const rclcpp::NodeOptions & options):Node("realsense_camera_node",options){

        buttonSubscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "button_value", 10, std::bind(&RealsenseCamera::savePointCloud, this, std::placeholders::_1));

        realsenseSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 10, std::bind(&RealsenseCamera::readCurrentPointCloud, this, std::placeholders::_1));

        centroidPublisher_ = this->create_publisher<std_msgs::msg::Float64>("centroidZ", 10);

        currentButtonValue_ = 0;
        previousButtonValue_ = 0;

        this->declare_parameter("point_cloud_path", "~/Documents");
        this->declare_parameter("number_of_captures", 10);

    }

    void RealsenseCamera::savePointCloud(std_msgs::msg::Bool msg){

        previousButtonValue_ = currentButtonValue_;
        currentButtonValue_ = msg.data;

        //If the capture button has been released (i.e falling edge 1 -> 0) we save the number of point clouds specified in the number_of_captures parameter tp pcd files

        if(currentButtonValue_ == 0 && previousButtonValue_ == 1){

            int numberOfCaptures = this->get_parameter("number_of_captures").as_int();

            for(int i = 0; i < numberOfCaptures; i++){
                pcl::PCLPointCloud2 pointCloudAsPcl_;
                writePointCloudtoFile(savedPointCloudAsPcl_);
            }   
        }
    }

    void RealsenseCamera::readCurrentPointCloud(sensor_msgs::msg::PointCloud2 msg){
        currentPointCloud_ = msg;
        pcl_conversions::toPCL(currentPointCloud_,savedPointCloudAsPcl_);
    }

    void RealsenseCamera::writePointCloudtoFile(pcl::PCLPointCloud2 pointcloud){
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

        pcl::io::savePCDFile(path,pointcloud,zero,identity,true);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(coral_cam::RealsenseCamera)
