# - Find Mininet library
# Find the native Mininet includes and library
# This module defines
#  MININET_INCLUDE_DIRS, where to find mininet.h, Set when
#                      MININET_INCLUDE_DIR is found.
#  MININET_LIBRARIES, libraries to link against to use Mininet.
#  MININET_ROOT_DIR, The base directory to search for Mininet.
#                  This can also be an environment variable.
#  MININET_FOUND, If false, do not try to use Mininet.
#
# also defined, but not for general use are
#  MININET_LIBRARY, where to find the Mininet library.

# If MININET_ROOT_DIR was defined in the environment, use it.
if(NOT MININET_ROOT_DIR AND NOT $ENV{MININET_ROOT_DIR} STREQUAL "")
  set(MININET_ROOT_DIR $ENV{MININET_ROOT_DIR})
endif()

set(_mininet_SEARCH_DIRS
  ${MININET_ROOT_DIR}
  /usr/local
  /usr/local/include
)

find_path(MININET_INCLUDE_DIR
  NAMES
    mininet
  HINTS
    ${_mininet_SEARCH_DIRS}
  PATH_SUFFIXES
    include
)

find_library(MININET_LIBRARY
  NAMES
    mininet
  HINTS
    ${_mininet_SEARCH_DIRS}
  PATH_SUFFIXES
    lib
  )

# handle the QUIETLY and REQUIRED arguments and set MININET_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(mininet DEFAULT_MSG
  MININET_INCLUDE_DIR MININET_LIBRARY)

if(MININET_FOUND)
  set(MININET_LIBRARIES ${MININET_LIBRARY})
  set(MININET_INCLUDE_DIRS ${MININET_INCLUDE_DIR})
endif(MININET_FOUND)

mark_as_advanced(
  MININET_INCLUDE_DIR
  MININET_LIBRARY
)
