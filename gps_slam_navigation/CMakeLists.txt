cmake_minimum_required(VERSION 3.5)
project(gps_slam_navigation)

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
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(gps_navigation_msgs REQUIRED)
find_package(PROJ REQUIRED CONFIG)

set(source_sets
  include/gps_slam_navigation/utils.hxx
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
  src/gps_slam_navigation/gps_slam_navigation.cxx
  src/main.cxx
)

set(dependencies
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  gps_navigation_msgs
  nav2_msgs
  PROJ
)

add_definitions(-DACCEPT_USE_OF_DEPRECATED_PROJ_API_H)

set(executable_name navigator)

add_executable(${executable_name}
  ${source_sets}
)

set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  ${source_sets}
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

target_link_libraries(
  ${executable_name}
  PROJ::proj
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
