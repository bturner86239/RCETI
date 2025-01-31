cmake_minimum_required(VERSION 3.5)

project(rceti_support)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
