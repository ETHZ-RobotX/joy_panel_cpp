cmake_minimum_required(VERSION 3.8)
project(joy_panel_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

# This is needed to handle Qt's MOC (Meta-Object Compiler) for signals/slots
set(CMAKE_AUTOMOC ON)

# Create a shared library from our source files
add_library(${PROJECT_NAME} SHARED
  src/autonomy_panel.cpp
  include/joy_panel_cpp/autonomy_panel.h
)

# Add include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link the library against its dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  sensor_msgs
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

# Register the plugin with pluginlib
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Installation rules
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rviz_common
  sensor_msgs
  pluginlib
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_package()