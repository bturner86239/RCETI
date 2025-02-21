# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rceti_support_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rceti_support_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rceti_support_FOUND FALSE)
  elseif(NOT rceti_support_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rceti_support_FOUND FALSE)
  endif()
  return()
endif()
set(_rceti_support_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rceti_support_FIND_QUIETLY)
  message(STATUS "Found rceti_support: 0.3.5 (${rceti_support_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rceti_support' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rceti_support_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rceti_support_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rceti_support_DIR}/${_extra}")
endforeach()
