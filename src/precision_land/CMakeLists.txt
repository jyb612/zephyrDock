cmake_minimum_required(VERSION 3.5)
project(precision_land)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)  # Add this line for the Trigger service
find_package(std_msgs REQUIRED)

# Include directories
include_directories(${Eigen3_INCLUDE_DIRS})

# Add executable
add_executable(precision_land
    PrecisionLand.cpp
)

# Link libraries
ament_target_dependencies(precision_land
  rclcpp
  Eigen3
  px4_ros2_cpp
  geometry_msgs
  tf2_ros
  tf2
  std_srvs  # Add this line for the Trigger service
)

# Set C++ standard
target_compile_features(precision_land PUBLIC c_std_99 cxx_std_17)

# Install the executable
install(TARGETS precision_land DESTINATION lib/${PROJECT_NAME})

# Install launch and config files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/)

# Linting and testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_package()