cmake_minimum_required(VERSION 3.8)
project(coral_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(coral_cam_node src/main.cpp src/coralCam.cpp src/button.cpp src/gui.cpp src/industrialCamera.cpp src/lights.cpp src/realsenseCamera.cpp)
ament_target_dependencies(coral_cam_node rclcpp std_msgs sensor_msgs)
target_link_libraries(coral_cam_node lgpio)

target_include_directories(coral_cam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(coral_cam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS coral_cam_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
