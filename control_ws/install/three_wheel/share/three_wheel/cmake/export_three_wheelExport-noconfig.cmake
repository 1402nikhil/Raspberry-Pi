#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "three_wheel::three_wheel" for configuration ""
set_property(TARGET three_wheel::three_wheel APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(three_wheel::three_wheel PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libthree_wheel.so"
  IMPORTED_SONAME_NOCONFIG "libthree_wheel.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS three_wheel::three_wheel )
list(APPEND _IMPORT_CHECK_FILES_FOR_three_wheel::three_wheel "${_IMPORT_PREFIX}/lib/libthree_wheel.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
