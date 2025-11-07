cmake_minimum_required(VERSION 3.8)
project(control_barrel_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(control_barrel_world src/control_barrel_world.cpp src/control_barrel_world_functions.cpp)
ament_target_dependencies(control_barrel_world rclcpp geometry_msgs vision_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS
  control_barrel_world
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

target_include_directories(control_barrel_world PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_package()
