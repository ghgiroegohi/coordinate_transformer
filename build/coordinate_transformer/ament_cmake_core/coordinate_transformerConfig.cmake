# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_coordinate_transformer_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED coordinate_transformer_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(coordinate_transformer_FOUND FALSE)
  elseif(NOT coordinate_transformer_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(coordinate_transformer_FOUND FALSE)
  endif()
  return()
endif()
set(_coordinate_transformer_CONFIG_INCLUDED TRUE)

# output package information
if(NOT coordinate_transformer_FIND_QUIETLY)
  message(STATUS "Found coordinate_transformer: 0.0.1 (${coordinate_transformer_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'coordinate_transformer' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT coordinate_transformer_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(coordinate_transformer_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${coordinate_transformer_DIR}/${_extra}")
endforeach()
