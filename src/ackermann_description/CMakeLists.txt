cmake_minimum_required(VERSION 3.8)
project(ackermann_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME})

# Permite que Gazebo resuelva package://ackermann_description/meshes/...
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
