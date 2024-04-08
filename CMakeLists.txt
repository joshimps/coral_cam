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
rclcpp_components_register_nodes(button_component "coral_cam::Button")
ament_target_dependencies(button_component rclcpp rclcpp_components std_msgs sensor_msgs)
target_link_libraries(button_component lgpio)


add_library(realsense_camera_component SHARED src/realsenseCamera.cpp)
rclcpp_components_register_nodes(realsense_camera_component "coral_cam::RealsenseCamera")
ament_target_dependencies(realsense_camera_component rclcpp_components rclcpp std_msgs sensor_msgs)


add_executable(main src/main.cpp)

target_link_libraries(main button_component realsense_camera_component lgpio)

ament_export_targets(export_button_component)
ament_export_targets(export_realsense_camera_component)


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

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
