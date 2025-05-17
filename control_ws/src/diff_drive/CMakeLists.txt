cmake_minimum_required(VERSION 3.8)
project(diff_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies      list all the dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  micro_ros_agent
)

# find dependencies           loop to declare dependency on the above package
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## Compile
add_library(
  diff_drive
  SHARED
  hardware/diffdrive_system.cpp
)
target_compile_features(diff_drive PUBLIC cxx_std_17)
target_include_directories(diff_drive PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/diff_drive>
)
ament_target_dependencies(
  diff_drive PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFF_DRIVE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diff_drive.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/diff_drive
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/diff_drive
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/diff_drive
)
install(TARGETS diff_drive
  EXPORT export_diff_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_diff_drive HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
