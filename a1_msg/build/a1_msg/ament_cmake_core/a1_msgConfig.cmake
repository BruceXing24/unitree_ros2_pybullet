# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_a1_msg_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED a1_msg_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(a1_msg_FOUND FALSE)
  elseif(NOT a1_msg_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(a1_msg_FOUND FALSE)
  endif()
  return()
endif()
set(_a1_msg_CONFIG_INCLUDED TRUE)

# output package information
if(NOT a1_msg_FIND_QUIETLY)
  message(STATUS "Found a1_msg: 0.0.0 (${a1_msg_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'a1_msg' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${a1_msg_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(a1_msg_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${a1_msg_DIR}/${_extra}")
endforeach()
