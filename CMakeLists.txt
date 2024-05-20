cmake_minimum_required(VERSION 3.8)
project(coral_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)

#Ros Standard Packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

#PCL
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

#QT5
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)

#Realsense
find_package(realsense2 REQUIRED)
find_package(realsense2_camera REQUIRED)

#Image Processing
find_package(image_proc REQUIRED)

#LGPIO
find_library(LGPIO_FOUND lgpio)

if(NOT LGPIO_FOUND)
        message(FATAL_ERROR "LGPIO NOT FOUND")
else()
        message("LGPIO FOUND")
endif()

#For QT5 Build
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)


#For PCL Build
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
)


add_library(helpers SHARED include/helpers.hpp)

add_library(battery_component SHARED src/battery.cpp)
rclcpp_components_register_nodes(battery_component "coral_cam::Battery")
ament_target_dependencies(battery_component rclcpp rclcpp_components std_msgs)

add_library(temperature_sensor_component SHARED src/temperature_sensor.cpp)
rclcpp_components_register_nodes(temperature_sensor_component "coral_cam::TemperatureSensor")
ament_target_dependencies(temperature_sensor_component rclcpp rclcpp_components std_msgs)

add_library(capture_button_component SHARED src/capture_button.cpp)
rclcpp_components_register_nodes(capture_button_component "coral_cam::CaptureButton")
ament_target_dependencies(capture_button_component rclcpp rclcpp_components std_msgs sensor_msgs)
target_link_libraries(capture_button_component lgpio)

add_library(realsense_camera_component SHARED src/real_sense_camera.cpp)
rclcpp_components_register_nodes(realsense_camera_component "coral_cam::RealSenseCamera")
ament_target_dependencies(realsense_camera_component rclcpp_components rclcpp std_msgs sensor_msgs pcl_conversions)
target_link_libraries(realsense_camera_component ${PCL_LIBRARIES} helpers)

add_library(industrial_camera_component SHARED src/industrial_camera.cpp)
rclcpp_components_register_nodes(industrial_camera_component "coral_cam::IndustrialCamera")
ament_target_dependencies(industrial_camera_component rclcpp_components rclcpp std_msgs sensor_msgs)

add_library(lights_component SHARED src/lights.cpp)
rclcpp_components_register_nodes(lights_component "coral_cam::Lights")
ament_target_dependencies(lights_component rclcpp_components rclcpp std_msgs sensor_msgs)

add_library(gpio_component SHARED src/gpio.cpp)
rclcpp_components_register_nodes(gpio_component "coral_cam::Gpio")
ament_target_dependencies(gpio_component rclcpp_components rclcpp std_msgs)
target_link_libraries(gpio_component lgpio )

add_executable(coral_cam src/coral_cam.cpp)
target_link_libraries(coral_cam battery_component capture_button_component gpio_component realsense_camera_component lights_component industrial_camera_component temperature_sensor_component) 

add_executable(gui include/gui.hpp src/gui.cpp)
ament_target_dependencies(gui rclcpp_components rclcpp sensor_msgs)
target_link_libraries(gui Qt5::Core Qt5::Quick Qt5::Widgets)

ament_export_targets(export_battery_component)
ament_export_targets(export_capture_button_component)
ament_export_targets(export_realsense_camera_component)
ament_export_targets(export_gpio_component)
ament_export_targets(export_temperature_sensor_component)
ament_export_targets(export_industrial_camera_component)
ament_export_targets(export_lights_component)
ament_export_targets(export_helpers)

install(TARGETS capture_button_component
        EXPORT export_capture_button_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS realsense_camera_component
        EXPORT export_realsense_camera_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS gpio_component
        EXPORT export_gpio_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS battery_component
        EXPORT export_battery_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS temperature_sensor_component
        EXPORT export_temperature_sensor_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS industrial_camera_component
        EXPORT export_industrial_camera_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS lights_component
        EXPORT export_lights_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


install(TARGETS helpers
        EXPORT export_helpers
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS
  gui
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
