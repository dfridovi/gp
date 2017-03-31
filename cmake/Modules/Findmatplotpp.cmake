# - Find Matplotpp library
# Find the native Matplotpp includes and library
# This module defines
#  MATPLOTPP_INCLUDE_DIRS, where to find matplotpp.h, Set when
#                      MATPLOTPP_INCLUDE_DIR is found.
#  MATPLOTPP_LIBRARIES, libraries to link against to use Matplotpp.
#  MATPLOTPP_ROOT_DIR, The base directory to search for Matplotpp.
#                  This can also be an environment variable.
#  MATPLOTPP_FOUND, If false, do not try to use Matplotpp.
#
# also defined, but not for general use are
#  MATPLOTPP_LIBRARY, where to find the Matplotpp library.

# If MATPLOTPP_ROOT_DIR was defined in the environment, use it.
if(NOT MATPLOTPP_ROOT_DIR AND NOT $ENV{MATPLOTPP_ROOT_DIR} STREQUAL "")
  set(MATPLOTPP_ROOT_DIR $ENV{MATPLOTPP_ROOT_DIR})
endif()

set(_matplotpp_SEARCH_DIRS
  ${MATPLOTPP_ROOT_DIR}
  /usr/local
  /usr/local/include
)

find_path(MATPLOTPP_INCLUDE_DIR
  NAMES
    matplotpp
  HINTS
    ${_matplotpp_SEARCH_DIRS}
  PATH_SUFFIXES
    include
)

find_library(MATPLOTPP_LIBRARY
  NAMES
    matplotpp
  HINTS
    ${_matplotpp_SEARCH_DIRS}
  PATH_SUFFIXES
    lib
  )

# handle the QUIETLY and REQUIRED arguments and set MATPLOTPP_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(matplotpp DEFAULT_MSG
  MATPLOTPP_INCLUDE_DIR MATPLOTPP_LIBRARY)

if(MATPLOTPP_FOUND)
  set(MATPLOTPP_LIBRARIES ${MATPLOTPP_LIBRARY})
  set(MATPLOTPP_INCLUDE_DIRS ${MATPLOTPP_INCLUDE_DIR})
endif(MATPLOTPP_FOUND)

mark_as_advanced(
  MATPLOTPP_INCLUDE_DIR
  MATPLOTPP_LIBRARY
)
