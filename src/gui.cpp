#include "gui.hpp"

namespace coral_cam{
   Gui::Gui(QWidget* parent):QWidget(parent), rclcpp::Node("gui_node"){
        
        rclcpp::QoS qosSettings(rclcpp::KeepLast(10),rmw_qos_profile_sensor_data);

        imageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_rect_resize", qosSettings, std::bind(&Gui::cameraFrameRecieved, this, std::placeholders::_1));
        
        centroidSubscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "/coral_cam/centroidZ",10, std::bind(&Gui::centroidFrameRecieved, this, std::placeholders::_1));

        this->setFixedSize(1920, 1080);

        mainLayout_ = new QVBoxLayout(this);
        
        // Create and position the Battery Label
        batteryLabel_ = new QLabel(this);
        batteryLabel_->setMinimumSize(1920,50);
        batteryLabel_->setMaximumSize(1920,50);
        batteryLabel_->setText("Battery:");
       
        mainLayout_->addWidget(batteryLabel_);

        // Create and position the Realsense Feed
        realsenseCameraFeed_ = new QLabel(this);
        realsenseCameraFeed_->setMinimumSize(960,540);
        realsenseCameraFeed_->setMaximumSize(960,540);
        mainLayout_->addWidget(realsenseCameraFeed_, 0 , Qt::AlignHCenter);

        centroidLabel_ = new QLabel(this);
        centroidLabel_->setMinimumSize(1920,50);
        centroidLabel_->setMaximumSize(1920,50);
        centroidLabel_->setText("Centroid Z: 0");
        
        this->show();
   }

   Gui::~Gui(){

   }

   void Gui::cameraFrameRecieved(sensor_msgs::msg::Image msg){
        QImage::Format format = QImage::Format_RGB888;
        QPixmap currentRealSenseCameraPixMap = QPixmap::fromImage(QImage(&msg.data[0], msg.width, msg.height, format));
        realsenseCameraFeed_->setPixmap(currentRealSenseCameraPixMap);
   }

   void Gui::centroidFrameRecieved(std_msgs::msg::Float64 msg){
        std::string text = "Centroid Z: " + std::to_string(msg.data);
        centroidLabel_->setText(QString::fromStdString(text));
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