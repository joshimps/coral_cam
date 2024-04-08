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


add_library(button_component SHARED src/button.cpp)
ament_target_dependencies(button_component rclcpp std_msgs sensor_msgs)
rclcpp_components_register_nodes(button_component "coralCam::Button")
ament_target_dependencies(button_component rclcpp rclcpp_components std_msgs sensor_msgs)

add_library(realsense_camera_component SHARED src/realsenseCamera.cpp)
ament_target_dependencies(realsense_camera_component rclcpp std_msgs sensor_msgs)
rclcpp_components_register_nodes(realsense_camera_component "coralCam::RealsenseCamera")
ament_target_dependencies(realsense_camera_component rclcpp_components rclcpp std_msgs)

add_executable(main src/main.cpp)

target_link_libraries(main button_component realsense_camera_component lgpio)


install(TARGETS 
        button_component
        realsense_camera_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
