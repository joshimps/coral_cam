#include "rclcpp/rclcpp.hpp"
#include "button.hpp"
#include "gui.hpp"
#include "industrialCamera.hpp"
#include "lights.hpp"
#include "realsenseCamera.hpp"
#include <lgpio.h>

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    rclcpp::NodeOptions buttonOptions;
    rclcpp::NodeOptions realsenseCameraOptions;

    const int buttonPinNumber = 13;
    const int ledPinNumber = 29;
    int gpioHandle;
    
    gpioHandle = lgGpiochipOpen(0);
    
    if(gpioHandle >= 0){
        auto button = std::make_shared<coral_cam::Button>(buttonOptions);
        button.get()->declare_parameter("gpio_handle",gpioHandle);
        executor.add_node(button);
    }

    auto realsenseCamera = std::make_shared<coral_cam::RealsenseCamera>(realsenseCameraOptions);
    executor.add_node(realsenseCamera);
    
    executor.spin();

    rclcpp::shutdown();

    return 1;
    
}