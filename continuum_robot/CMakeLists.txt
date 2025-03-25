cmake_minimum_required(VERSION 3.5)
project(continuum_robot)

# Add the correct path for header files
include_directories(${PROJECT_SOURCE_DIR}/include)
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

#target_link_libraries(interface
#  tf2_geometry_msgs::tf2_geometry_msgs
#)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(interactive_markers REQUIRED)

add_library(${PROJECT_NAME} SHARED src/Continuum.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  interactive_markers
)

add_executable(interface src/interface.cpp)
ament_target_dependencies(interface
  rclcpp
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  interactive_markers
)
target_link_libraries(interface ${PROJECT_NAME})

add_executable(core_node src/core_node.cpp)
ament_target_dependencies(core_node
  rclcpp
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  interactive_markers
)
target_link_libraries(core_node ${PROJECT_NAME})

add_executable(create_cont_robot src/create_cont_robot.cpp)
ament_target_dependencies(create_cont_robot
  rclcpp
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  interactive_markers
)
target_link_libraries(create_cont_robot ${PROJECT_NAME})

install(TARGETS
  interface
  core_node
  create_cont_robot
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Set RPATH for executables
set_target_properties(interface core_node create_cont_robot PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

ament_package()