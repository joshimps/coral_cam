#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <ctime>

namespace coral_cam{
    class RealsenseCamera: public rclcpp::Node{
        public:

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Constructors and Destructors                                                                          //
            //////////////////////////////////////////////////////////////////////////////////////////////////////////

            RealsenseCamera(const rclcpp::NodeOptions & options);

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Public Methods                                                                                        //
            //////////////////////////////////////////////////////////////////////////////////////////////////////////

            /**
            Saves the current point cloud seen by the Intel Realsense camera
            
            */
            void savePointCloud(std_msgs::msg::Bool msg);

            /**
            Writes the currently held point cloud by currentPointCloud_ to a .pcd file

            */
            void writePointCloudtoFile();

            /**
            Returns the current point cloud save in currentPointCloud_;
            @return capture successful
            */
            void readCurrentPointCloud(sensor_msgs::msg::PointCloud2 msg);

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
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr realsenseSubscriber_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr buttonSubscriber_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr capturedPointCloudPublisher_;

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Constants                                                                                             //
            //////////////////////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Variables                                                                                             //
            //////////////////////////////////////////////////////////////////////////////////////////////////////////

            sensor_msgs::msg::PointCloud2 savedPointCloud_;
            pcl::PCLPointCloud2 savedPointCloudAsPcl_;
            sensor_msgs::msg::PointCloud2 currentPointCloud_;
    };
}