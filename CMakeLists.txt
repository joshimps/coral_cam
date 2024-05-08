cmake_minimum_required(VERSION 3.8)
project(coral_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)

find_package(std_msgs REQUIRED)
find_package(image_proc REQUIRED)

find_library(lgpio)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
)

add_library(button_component SHARED src/button.cpp)
rclcpp_components_register_nodes(button_component "coral_cam::Button")
ament_target_dependencies(button_component rclcpp rclcpp_components std_msgs sensor_msgs)
target_link_libraries(button_component lgpio)

add_library(realsense_camera_component SHARED src/realsenseCamera.cpp)
rclcpp_components_register_nodes(realsense_camera_component "coral_cam::RealsenseCamera")
ament_target_dependencies(realsense_camera_component rclcpp_components rclcpp std_msgs sensor_msgs pcl_conversions)
target_link_libraries(realsense_camera_component ${PCL_LIBRARIES})

add_library(gpio_component SHARED src/gpio.cpp)
rclcpp_components_register_nodes(gpio_component "coral_cam::Gpio")
ament_target_dependencies(gpio_component rclcpp_components rclcpp std_msgs)
target_link_libraries(gpio_component lgpio )

add_executable(coral_cam src/coralCam.cpp)
target_link_libraries(coral_cam realsense_camera_component button_component gpio_component) 

add_executable(gui include/gui.hpp src/gui.cpp)
ament_target_dependencies(gui rclcpp_components rclcpp sensor_msgs)
target_link_libraries(gui Qt5::Core Qt5::Quick Qt5::Widgets)

ament_export_targets(export_button_component)
ament_export_targets(export_realsense_camera_component)
ament_export_targets(export_gpio_component)

install(TARGETS button_component
        EXPORT export_button_component
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

install(TARGETS
  gui
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
