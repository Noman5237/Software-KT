cmake_minimum_required(VERSION 3.8)
project(temperature_control_systems)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(temperature_control_systems_interfaces REQUIRED)

add_executable(temperature_systems_controller src/temperature_systems_controller.cpp)

ament_target_dependencies(
	temperature_systems_controller 
	rclcpp std_msgs 
	temperature_control_systems_interfaces)

install(TARGETS
	temperature_systems_controller 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
