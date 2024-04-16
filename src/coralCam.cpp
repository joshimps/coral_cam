#include "rclcpp/rclcpp.hpp"
#include "button.hpp"
#include "gui.hpp"
#include "industrialCamera.hpp"
#include "lights.hpp"
#include "realsenseCamera.hpp"
#include "gpio.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions buttonOptions;
    rclcpp::NodeOptions realsenseCameraOptions;
    rclcpp::NodeOptions gpioOptions;

    auto button = std::make_shared<coral_cam::Button>(buttonOptions);
    executor.add_node(button);

    auto realsenseCamera = std::make_shared<coral_cam::RealsenseCamera>(realsenseCameraOptions);
    executor.add_node(realsenseCamera);

    auto gpio = std::make_shared<coral_cam::Gpio>(gpioOptions);
    executor.add_node(gpio);

    executor.spin();

    rclcpp::shutdown();

    return 1;
    
}