cmake_minimum_required(VERSION 3.5)
project(gts_navigator)

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
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(gts_navigation_msgs REQUIRED)
find_package(PROJ REQUIRED CONFIG)

set(source_sets
  include/gts_navigator/utils.hxx
  src/gts_navigator/gts_navigator.cxx
  src/main.cxx
)

set(dependencies
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  gts_navigation_msgs
  nav2_msgs
  PROJ
)

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
