cmake_minimum_required(VERSION 3.8)
project(pcl_obstacle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pcl_obstacle_detector src/pcl_obstacle_detector.cpp src/pcl_obstacle_detector_functions.cpp)
ament_target_dependencies(pcl_obstacle_detector rclcpp sensor_msgs vision_msgs PCL pcl_conversions std_msgs)

install(TARGETS
  pcl_obstacle_detector
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(pcl_obstacle_detector PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_package()
