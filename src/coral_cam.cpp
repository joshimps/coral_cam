#include "rclcpp/rclcpp.hpp"
#include "battery.hpp"
#include "capture_button.hpp"
#include "gui.hpp"
#include "industrial_camera.hpp"
#include "lights.hpp"
#include "real_sense_camera.hpp"
#include "gpio.hpp"
#include "temperature_sensor.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    rclcpp::NodeOptions battery_options;
    rclcpp::NodeOptions capture_button_options;
    rclcpp::NodeOptions real_sense_camera_options;
    rclcpp::NodeOptions industrial_camera_options;
    rclcpp::NodeOptions lights_options;
    rclcpp::NodeOptions gpio_options;
    rclcpp::NodeOptions gui_options;
    rclcpp::NodeOptions temperature_sensor_options;

    auto battery = std::make_shared<coral_cam::Battery>(battery_options);
    executor.add_node(battery);

    auto temperature_sensor = std::make_shared<coral_cam::TemperatureSensor>(temperature_sensor_options);
    executor.add_node(temperature_sensor);

    auto capture_button = std::make_shared<coral_cam::CaptureButton>(capture_button_options);
    executor.add_node(capture_button);

    auto real_sense_camera = std::make_shared<coral_cam::RealSenseCamera>(real_sense_camera_options);
    executor.add_node(real_sense_camera);

    auto gpio = std::make_shared<coral_cam::Gpio>(gpio_options);
    executor.add_node(gpio);

    auto industrial_camera = std::make_shared<coral_cam::IndustrialCamera>(industrial_camera_options);
    executor.add_node(industrial_camera);

    auto lights = std::make_shared<coral_cam::Lights>(lights_options);
    executor.add_node(lights);

    while (rclcpp::ok())
    {
        try
        {
            executor.spin_some();
        }
        catch (const std::exception &e)
        {
            rclcpp::shutdown();
        }
    }

    rclcpp::shutdown();

    return 1;
}