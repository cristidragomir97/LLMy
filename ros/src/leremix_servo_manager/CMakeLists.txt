cmake_minimum_required(VERSION 3.10)
project(leremix_servo_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# Include directories
include_directories(include)

# Build MotorManager library
add_library(motor_manager src/MotorManager.cpp)
target_include_directories(motor_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Ping test executable
add_executable(ping_test src/ping_test.cpp)
target_link_libraries(ping_test motor_manager)
ament_target_dependencies(ping_test rclcpp)

# Servo manager node
add_executable(servo_manager_node src/servo_manager_node.cpp)
target_link_libraries(servo_manager_node motor_manager)
ament_target_dependencies(servo_manager_node
  rclcpp
  std_msgs
  sensor_msgs
  rclcpp_components)

# Install executables
install(TARGETS
  ping_test
  servo_manager_node
  DESTINATION lib/${PROJECT_NAME})

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()