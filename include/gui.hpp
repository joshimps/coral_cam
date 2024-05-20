#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QPushButton>
#include <QImage>
#include <QLabel>
#include <QSizePolicy>

#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QTabWidget>

#include <QPixmap>
#include <QString>

#include <string>
#include <bits/stdc++.h>

namespace coral_cam
{
    class Gui : public QTabWidget, public rclcpp::Node
    {
        Q_OBJECT
    public:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constructors and Destructors                                                                          //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        Gui(const rclcpp::NodeOptions &options, QTabWidget *parent);
        ~Gui();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Public Methods                                                                                        //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

    private:
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Callbacks                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        void SetCameraFrame(sensor_msgs::msg::Image::SharedPtr msg);
        void SetTemperature(std_msgs::msg::String::SharedPtr msg);
        void SetBattery(std_msgs::msg::String::SharedPtr msg);
        void SetCaptureStatus(std_msgs::msg::Bool::SharedPtr msg);
        void SetCentreDistance(std_msgs::msg::Int64::SharedPtr msg);



        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Private Methods                                                                                       //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Node, Publishers and Subscribers                                                                      //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr temperature_subscriber_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr battery_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr capture_status_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr centre_distance_subscriber_;



        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Constants                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Variables                                                                                             //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////

        QTabWidget *main_layout_;

        QWidget *pam_tab_;
        QGridLayout *pam_tab_grid_layout_;

        QHBoxLayout *pam_header_bar_;
        QLabel *current_temp_;
        QLabel *current_battery_;

        QLabel *camera_feed_;

        QHBoxLayout *pam_footer_bar_;
        QLabel *depth_to_center_;
        QLabel *capture_status_;

        QWidget *settings_tab_;
        QGridLayout *settings_tab_grid_layout_;
    };
}
