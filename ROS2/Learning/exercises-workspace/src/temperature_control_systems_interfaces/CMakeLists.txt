cmake_minimum_required(VERSION 3.8)
project(temperature_control_systems_interfaces)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCurrentTemperature.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
