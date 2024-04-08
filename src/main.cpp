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

    buttonOptions.allow_undeclared_parameters(true);
    buttonOptions.automatically_declare_parameters_from_overrides(true);

    int gpioHandle;
    
    

    auto button = std::make_shared<coral_cam::Button>(buttonOptions);
    button.get()->set_parameter(rclcpp::Parameter("gpio_handle", 69));
    executor.add_node(button);
    
    auto realsenseCamera = std::make_shared<coral_cam::RealsenseCamera>(realsenseCameraOptions);
    executor.add_node(realsenseCamera);
    
    executor.spin();

    rclcpp::shutdown();

    return 1;
    
}