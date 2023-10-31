cmake_minimum_required(VERSION 3.5)
project(gts_navigation_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GoalWaypoints.msg"
  "msg/NavigationStatusStamped.msg"
  "msg/NavigationResultStamped.msg"
  "srv/GoalCancel.srv"
  DEPENDENCIES std_msgs geometry_msgs action_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_generators)

ament_package()