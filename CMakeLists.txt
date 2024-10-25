CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
project(srf_laser_odometry)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

foreach(pkg IN LISTS dependencies)
  find_package(${pkg} REQUIRED)
endforeach()

include_directories(include)

add_library(laser_odometry
  src/utilities/laser_odometry_refscans.cpp
)
set(libraries laser_odometry)

file(GLOB executable_files src/nodes/*.cpp)
foreach(executable_file ${executable_files})
  get_filename_component(executable_name ${executable_file} NAME_WE)
  add_executable(${executable_name} ${executable_file})
  ament_target_dependencies(${executable_name} ${dependencies})
  install(TARGETS ${executable_name} DESTINATION lib/${PROJECT_NAME})
endforeach()

target_link_libraries(srf_node ${libraries})

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies})

ament_package()