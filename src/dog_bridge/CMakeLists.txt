cmake_minimum_required(VERSION 3.8)
project(dog_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(bspline REQUIRED)
# SCAN's B-spline message, so the bridge can hand RACER's 3D trajectory to
# open_loop_controller directly (multi-floor ramp following).
find_package(scan_planner_msgs REQUIRED)

add_executable(dog_bridge_node src/dog_bridge_node.cpp)
ament_target_dependencies(dog_bridge_node rclcpp geometry_msgs nav_msgs sensor_msgs bspline scan_planner_msgs)

add_executable(task_node src/task_node.cpp)
ament_target_dependencies(task_node rclcpp geometry_msgs nav_msgs std_msgs)

add_executable(cloud_accumulator src/cloud_accumulator.cpp)
ament_target_dependencies(cloud_accumulator rclcpp sensor_msgs)

install(TARGETS dog_bridge_node task_node cloud_accumulator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
