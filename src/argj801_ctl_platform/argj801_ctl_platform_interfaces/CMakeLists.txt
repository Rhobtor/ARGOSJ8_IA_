cmake_minimum_required(VERSION 3.8)
project(argj801_ctl_platform_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SetVelocity.srv"
    "srv/GetVelocity.srv"
    "srv/EmergencyStop.srv"
    "msg/CmdThrottleMsg.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
