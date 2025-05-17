#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "diff_drive::diff_drive" for configuration ""
set_property(TARGET diff_drive::diff_drive APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(diff_drive::diff_drive PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdiff_drive.so"
  IMPORTED_SONAME_NOCONFIG "libdiff_drive.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS diff_drive::diff_drive )
list(APPEND _IMPORT_CHECK_FILES_FOR_diff_drive::diff_drive "${_IMPORT_PREFIX}/lib/libdiff_drive.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
