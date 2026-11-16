cmake_minimum_required(VERSION 3.8)
project(pan_tilt_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(urdf REQUIRED)

set(DEPS rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs builtin_interfaces urdf)

# Shared helpers: joint states, limits from robot_description, trajectory execution, Ctrl+C cancel
add_library(pan_tilt_client src/pan_tilt_client.cpp)
target_include_directories(pan_tilt_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pan_tilt_client ${DEPS})

foreach(exe move_camera circle_demo sine_demo)
  add_executable(${exe} src/${exe}.cpp)
  target_link_libraries(${exe} pan_tilt_client)
  ament_target_dependencies(${exe} ${DEPS})
endforeach()

install(TARGETS pan_tilt_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS move_camera circle_demo sine_demo
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY config launch rviz urdf meshes
  DESTINATION share/${PROJECT_NAME})

ament_package()
