cmake_minimum_required(VERSION 3.8)
project(pan_tilt_camera)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz urdf meshes
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/move_camera.py scripts/circle_demo.py scripts/sine_demo.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
