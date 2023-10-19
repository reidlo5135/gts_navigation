cmake_minimum_required(VERSION 3.5)
project(gps_slam_converter)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
  /usr/local/include
  /usr/local/include/proj
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_slam_conversion_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(PROJ REQUIRED CONFIG)

add_definitions(-DACCEPT_USE_OF_DEPRECATED_PROJ_API_H)

set(SOURCES
  include/gps_slam_converter/utils.hxx
  src/conversions/gps_to_slam.cxx
  src/conversions/slam_to_gps.cxx
  src/points/geo_point.cxx
  src/points/gts_point.cxx
  src/points/ktm_point.cxx
  src/points/quaternion_point.cxx
  src/points/slam_point.cxx
  src/math/common_math.cxx
  src/math/gps_math.cxx
  src/math/gts_math.cxx
  src/math/ktm_math.cxx
  src/math/slam_math.cxx
  src/math/slam_point_generator.cxx
  src/gps_slam_converter/gps_slam_converter.cxx
  src/main.cxx
)

set(DEPENDS
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  gps_slam_conversion_msgs
  builtin_interfaces
  PROJ
)

add_executable(converter
  ${SOURCES}
)

ament_target_dependencies(converter
  ${DEPENDS}
)

install(TARGETS
  converter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
