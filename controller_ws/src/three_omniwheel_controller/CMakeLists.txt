cmake_minimum_required(VERSION 3.8)
project(three_omniwheel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


generate_parameter_library(three_omniwheel_controller_parameters
  src/three_omniwheel_controller_parameter.yaml
)

add_library(three_omniwheel_controller SHARED
  src/three_omniwheel_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(three_omniwheel_controller PUBLIC cxx_std_17)
target_include_directories(three_omniwheel_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/three_omniwheel_controller>
)

target_link_libraries(three_omniwheel_controller PUBLIC three_omniwheel_controller_parameters)
ament_target_dependencies(three_omniwheel_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_compile_definitions(three_omniwheel_controller PRIVATE "THREE_OMNIWHEEL_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface three_omniwheel_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(controller_manager REQUIRED)
endif()

install(
  DIRECTORY include/
  DESTINATION include/three_omniwheel_controller
)
install(TARGETS three_omniwheel_controller three_omniwheel_controller_parameters
  EXPORT export_three_omniwheel_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_three_omniwheel_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
