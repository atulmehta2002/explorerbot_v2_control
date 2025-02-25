cmake_minimum_required(VERSION 3.8)
project(explorerbot_v2_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find WiringPi manually instead of using pkg-config
find_library(WIRINGPI_LIB wiringPi REQUIRED)

# Ensure correct include and library paths
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# Create executable
add_executable(motor_controller src/motor_controller.cpp)

# Link against WiringPi and ROS 2 libraries
target_link_libraries(motor_controller ${WIRINGPI_LIB} wiringPi)
ament_target_dependencies(motor_controller rclcpp geometry_msgs)

# Install executable
install(TARGETS
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

