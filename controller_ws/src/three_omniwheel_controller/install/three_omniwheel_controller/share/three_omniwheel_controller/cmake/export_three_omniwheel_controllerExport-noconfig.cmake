#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "three_omniwheel_controller::three_omniwheel_controller" for configuration ""
set_property(TARGET three_omniwheel_controller::three_omniwheel_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(three_omniwheel_controller::three_omniwheel_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libthree_omniwheel_controller.so"
  IMPORTED_SONAME_NOCONFIG "libthree_omniwheel_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS three_omniwheel_controller::three_omniwheel_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_three_omniwheel_controller::three_omniwheel_controller "${_IMPORT_PREFIX}/lib/libthree_omniwheel_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
