cmake_minimum_required(VERSION 3.12)

project(rceti_deployment)

find_package(ament_cmake REQUIRED)
ament_package()

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME})
