cmake_minimum_required(VERSION 2.8.7)
project(gp C CXX)

# Build options.
option(BUILD_TESTS "Build tests" ON)
option(BUILD_DOCUMENTATION "Build documentation" OFF)
set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations")

# Extra variables.
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  add_definitions(-DSYSTEM_OSX)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  add_definitions(-DSYSTEM_LINUX)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

add_definitions(-DGP_DATA_DIR="${CMAKE_SOURCE_DIR}/data/")

# Add cmake modules.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)
message("Cmake module path: ${CMAKE_MODULE_PATH}")

# Add cmake utilities.
include(cmake/Utils.cmake)

# Add dependencies.
include(cmake/Dependencies.cmake)

# Check for C++11 features and enable.
gp_enable_cpp11()

# Set the build type. Default to Release mode.
if(NOT CMAKE_BUILD_TYPE)
  message("Defaulting to building targets in Release mode.")
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# Find and build third party libraries.
add_subdirectory(external)

# Add the source directories to the search path so cmake can find our headers.
include_directories(${CMAKE_SOURCE_DIR}/include/)

# Collect source files and build the berkeley_sfm object.
file(GLOB_RECURSE gp_srcs ${CMAKE_SOURCE_DIR}/src/*.cpp)
add_library(gp ${gp_srcs})

# Set up output directory for executables.
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin/)

# Find and build executables.
add_subdirectory(exec)

# Find and build tests.
if (BUILD_TESTS)
  message("Build tests is enabled.")
  add_subdirectory(test)
endif (BUILD_TESTS)

# Find and build documentation.
#if (BUILD_DOCUMENTATION)
#  message("Build documentation is enabled.")
#  add_subdirectory(documentation)
#endif (BUILD_DOCUMENTATION)
