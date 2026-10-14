
cmake_minimum_required(VERSION 3.8)
project(rviz_focal_control)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/focal_panel.cpp
  include/rviz_focal_control/focal_panel.hpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  std_msgs
  rviz_common
  pluginlib
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(
  rviz_common plugin_description.xml
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS scripts/keyboard_focal_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_focal_control
)

ament_package()
