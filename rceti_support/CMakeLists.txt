# SPDX-License-Identifier: Apache-2.0
# CMake file for rceti_support, a package for the RCETI project.
# This package is responsible for the support of the RCETI Frame URDF and it's components
# DO NOT EDIT THIS CMAKE FILE, NOTHING WILL NEED TO BE CHANGED HERE UNLESS MODIFYING PACKAGE STUCTURE
cmake_minimum_required(VERSION 3.5)

project(rceti_support)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
