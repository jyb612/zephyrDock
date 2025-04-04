cmake_minimum_required(VERSION 3.5)
project(precision_land_test)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Build the node
add_executable(test_subscriber src/main.cpp src/precision_land_test.cpp)
ament_target_dependencies(test_subscriber rclcpp geometry_msgs)

# Install
install(TARGETS test_subscriber DESTINATION lib/${PROJECT_NAME})

# Required for ament
ament_package()
