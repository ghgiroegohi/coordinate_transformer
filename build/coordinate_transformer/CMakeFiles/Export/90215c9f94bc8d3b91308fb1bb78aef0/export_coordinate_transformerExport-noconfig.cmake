#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "coordinate_transformer::coordinate_transformer" for configuration ""
set_property(TARGET coordinate_transformer::coordinate_transformer APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(coordinate_transformer::coordinate_transformer PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcoordinate_transformer.so"
  IMPORTED_SONAME_NOCONFIG "libcoordinate_transformer.so"
  )

list(APPEND _cmake_import_check_targets coordinate_transformer::coordinate_transformer )
list(APPEND _cmake_import_check_files_for_coordinate_transformer::coordinate_transformer "${_IMPORT_PREFIX}/lib/libcoordinate_transformer.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
