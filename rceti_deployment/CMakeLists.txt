# SPDX-License-Identifier: Apache-2.0
# CMake file for rceti_deployment, a package for the RCETI project.
# This package is responsible for deployment configurations and launch files.
# DO NOT EDIT THIS CMAKE FILE, NOTHING WILL NEED TO BE CHANGED HERE UNLESS MODIFYING PACKAGE STUCTURE
cmake_minimum_required(VERSION 3.12)

project(rceti_deployment)

find_package(ament_cmake REQUIRED)
ament_package()

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME})
