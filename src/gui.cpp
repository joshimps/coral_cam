#include "gui.hpp"

namespace coral_cam
{
    Gui::Gui(const rclcpp::NodeOptions &options, QTabWidget *parent) : QTabWidget(parent), rclcpp::Node("gui_node",options)
    {

        rclcpp::QoS qos_settings(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);

        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/real_sense/color/image_rect_resized", qos_settings, std::bind(&Gui::SetCameraFrame, this, std::placeholders::_1));

        temperature_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/coral_cam/current_temperature", 10, std::bind(&Gui::SetTemperature, this, std::placeholders::_1));

        battery_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/coral_cam/current_battery", 10, std::bind(&Gui::SetBattery, this, std::placeholders::_1));

        capture_status_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/coral_cam/capture_in_progress", 10, std::bind(&Gui::SetCaptureStatus, this, std::placeholders::_1));
        
        centre_distance_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "/coral_cam/centre_distance", 10, std::bind(&Gui::SetCentreDistance, this, std::placeholders::_1));


        // Create the first tab where our pam display will go
        QSizePolicy expandingSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        // Create a tab for the pam display to go
        pam_tab_ = new QWidget();
        this->addTab(pam_tab_, QString("PAM"));

        pam_tab_grid_layout_ = new QGridLayout(pam_tab_);
        pam_tab_grid_layout_->setRowStretch(0, 12.5);
        pam_tab_grid_layout_->setRowStretch(1, 75);
        pam_tab_grid_layout_->setRowStretch(2, 12.5);

        pam_tab_grid_layout_->setColumnStretch(0, 1);
        pam_tab_grid_layout_->setColumnStretch(1, 1);
        pam_tab_grid_layout_->setColumnStretch(2, 1);

        // Create and position the header display

        pam_header_bar_ = new QHBoxLayout();
        pam_header_bar_->setStretch(0, 50);
        pam_header_bar_->setStretch(1, 50);
        pam_tab_grid_layout_->addLayout(pam_header_bar_, 0, 0);

        current_temp_ = new QLabel();
        current_temp_->setText("Current Temperature: Loading");
        current_temp_->setAlignment(Qt::AlignLeft);
        pam_header_bar_->addWidget(current_temp_);

        current_battery_ = new QLabel();
        current_battery_->setText("Current Battery: Loading");
        pam_header_bar_->addWidget(current_battery_);
        current_battery_->setAlignment(Qt::AlignRight);

        // Create and position the Realsense Feed
        camera_feed_ = new QLabel();
        camera_feed_->setSizePolicy(expandingSizePolicy);
        pam_tab_grid_layout_->addWidget(camera_feed_, 1, 0);

        // Create and position the footer display
        pam_footer_bar_ = new QHBoxLayout();
        pam_footer_bar_->setStretch(0, 50);
        pam_footer_bar_->setStretch(1, 50);
        pam_tab_grid_layout_->addLayout(pam_footer_bar_, 2, 0);

        depth_to_center_ = new QLabel();
        depth_to_center_->setText("Depth To Center: ");
        depth_to_center_->setAlignment(Qt::AlignLeft);
        pam_footer_bar_->addWidget(depth_to_center_);

        capture_status_ = new QLabel();
        capture_status_->setText("Capture Status: Awaiting Capture");
        capture_status_->setStyleSheet("QLabel { color : green; }");
        capture_status_->setAlignment(Qt::AlignRight);
        pam_footer_bar_->addWidget(capture_status_);

        // Create a tab for our settings to go
        settings_tab_ = new QWidget();
        this->addTab(settings_tab_, QString("Settings"));
        settings_tab_grid_layout_ = new QGridLayout(settings_tab_);

        this->show();
        this->showFullScreen();
    }

    Gui::~Gui()
    {
    }

    void Gui::SetCameraFrame(sensor_msgs::msg::Image::SharedPtr msg)
    {
        QImage::Format format = QImage::Format_RGB888;
        QPixmap current_real_sense_camera_pix_map = QPixmap::fromImage(QImage(&msg->data[0], msg->width, msg->height, format));
        camera_feed_->setPixmap(current_real_sense_camera_pix_map);
    }

    void Gui::SetTemperature(std_msgs::msg::String::SharedPtr msg)
    {
        std::string text = "Current Temperature: " + msg->data;
        current_temp_->setText(text.c_str());
    }

    void Gui::SetBattery(std_msgs::msg::String::SharedPtr msg)
    {
        std::string text = "Current Battery: " + msg->data;
        current_battery_->setText(text.c_str());
        
    }

    void Gui::SetCaptureStatus(std_msgs::msg::Bool::SharedPtr msg)
    {
        std::string text;

        if(msg->data){
            text = "Capture Status: In Progress";
            capture_status_->setStyleSheet("QLabel { color : red; }");
        }
        else{
            text = "Capture Status: Awaiting Capture";
            capture_status_->setStyleSheet("QLabel { color : green; }");
        }
        capture_status_->setText(text.c_str());
        
    }

    void Gui::SetCentreDistance(std_msgs::msg::Int64::SharedPtr msg)
    {

        std::string text;

        if(msg->data == 0){
            text = "Depth To Center: UNKNOWN";
            depth_to_center_->setStyleSheet("QLabel { color : red; }");
        }
        else if(msg->data >= 25 || msg->data <=50){
            text = "Depth To Center: " + std::to_string(msg->data) + "mm";
            depth_to_center_->setStyleSheet("QLabel { color : green; }");
        }
        else if(msg->data > 50){
            text = "Depth To Center: " + std::to_string(msg->data) + "mm";
            depth_to_center_->setStyleSheet("QLabel { color : red; }");
        }

        depth_to_center_->setText(text.c_str());
    }


}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions gui_options;
    gui_options.use_intra_process_comms(true);

    QTabWidget *parent = 0;
    auto gui = std::make_shared<coral_cam::Gui>(gui_options,parent);
    executor.add_node(gui);

    app.processEvents();

    while (rclcpp::ok())
    {
        app.processEvents();
        executor.spin_some();
    }

    executor.remove_node(gui);
    rclcpp::shutdown();
    QApplication::quit();

    return 1;
}