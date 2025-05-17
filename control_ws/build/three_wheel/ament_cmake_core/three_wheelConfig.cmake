# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_three_wheel_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED three_wheel_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(three_wheel_FOUND FALSE)
  elseif(NOT three_wheel_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(three_wheel_FOUND FALSE)
  endif()
  return()
endif()
set(_three_wheel_CONFIG_INCLUDED TRUE)

# output package information
if(NOT three_wheel_FIND_QUIETLY)
  message(STATUS "Found three_wheel: 0.0.0 (${three_wheel_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'three_wheel' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${three_wheel_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(three_wheel_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${three_wheel_DIR}/${_extra}")
endforeach()
