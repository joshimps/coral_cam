#include "gui.hpp"

namespace coral_cam{
   Gui::Gui(QWidget* parent):QWidget(parent), rclcpp::Node("gui_node"){
        
        rclcpp::QoS qos_settings(rclcpp::KeepLast(10),rmw_qos_profile_sensor_data);

        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/real_sense/color/image_rect_resized", qos_settings, std::bind(&Gui::CameraFrameRecieved, this, std::placeholders::_1));
        

        this->setFixedSize(1920, 1080);

        main_layout_ = new QVBoxLayout(this);
        
        
        // Create and position the Realsense Feed
        realsense_camera_feed_ = new QLabel(this);
        realsense_camera_feed_->setMinimumSize(960,540);
        realsense_camera_feed_->setMaximumSize(960,540);
        main_layout_->addWidget(realsense_camera_feed_, 0 , Qt::AlignHCenter);
        
        this->show();
   }

   Gui::~Gui(){

   }

   void Gui::CameraFrameRecieved(sensor_msgs::msg::Image msg){
        QImage::Format format = QImage::Format_RGB888;
        QPixmap current_real_sense_camera_pix_map = QPixmap::fromImage(QImage(&msg.data[0], msg.width, msg.height, format));
        realsense_camera_feed_->setPixmap(current_real_sense_camera_pix_map);
   }

}

int main(int argc, char * argv[]){
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    QWidget* parent = 0;
    auto gui = std::make_shared<coral_cam::Gui>(parent);
    executor.add_node(gui);

    app.processEvents();

    while (rclcpp::ok())
    {
        app.processEvents();
        executor.spin_some();
    }

    executor.remove_node(gui);

    rclcpp::shutdown();

    return 1;
    
}