#include "gui.hpp"

namespace coral_cam{
   Gui::Gui(QWidget* parent):QWidget(parent), rclcpp::Node("gui_node"){
        
        rclcpp::QoS qosSettings(rclcpp::KeepLast(10),rmw_qos_profile_sensor_data);

        imageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_rect_resize", qosSettings, std::bind(&Gui::cameraFrameRecieved, this, std::placeholders::_1));

        this->setFixedSize(1920, 1080);
        // Create and position the button
        realsenseCameraFeed_ = new QLabel(this);
        realsenseCameraFeed_->setGeometry(0,0,1920,1080);
        this->show();
   }

   Gui::~Gui(){

   }

   void Gui::cameraFrameRecieved(sensor_msgs::msg::Image msg){
        QImage::Format format = QImage::Format_RGB888;
        QPixmap currentRealSenseCameraPixMap = QPixmap::fromImage(QImage(&msg.data[0], msg.width, msg.height, format));
        realsenseCameraFeed_->setPixmap(currentRealSenseCameraPixMap);

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