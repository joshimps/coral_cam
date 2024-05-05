#include "gui.hpp"

namespace coral_cam{
   Gui::Gui(QWidget* parent):QWidget(parent), rclcpp::Node("gui_node"){
      RCLCPP_INFO(this->get_logger(), "YO");
      setFixedSize(100, 50);
      // Create and position the button
      publishButton_ = new QPushButton("Hello World", this);
      publishButton_->setGeometry(10, 10, 80, 30);
   }

   Gui::~Gui(){

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
    gui->show();

    while (rclcpp::ok())
    {
        app.processEvents();
        executor.spin_some();
    }

    executor.remove_node(gui);

    rclcpp::shutdown();

    return 1;
    
}