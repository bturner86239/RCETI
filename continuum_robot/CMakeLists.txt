cmake_minimum_required(VERSION 3.5)
project(continuum_robot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
)

# Build core_node
add_executable(core_node
  src/core_node.cpp
  src/Continuum.cpp
  src/interface.cpp
)
ament_target_dependencies(core_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  ament_index_cpp
  interactive_markers
)

# Build continuum_interface_node
add_executable(continuum_interface_node
  src/continuum_interface_node.cpp
  src/Continuum.cpp
  src/interface.cpp
)
ament_target_dependencies(continuum_interface_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs 
  visualization_msgs
  ament_index_cpp
  interactive_markers
)

# Install targets
install(TARGETS
  core_node
  continuum_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
