# Google gtest for unit testing.
add_subdirectory(gtest)
include_directories(${gtest_SOURCE_DIR} ${gtest_INCLUDE_DIR})

# QCustomPlot for plotting.
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  add_subdirectory(matplotpp)
  include_directories(${matplotpp_SOURCE_DIR} ${matplotpp_INCLUDE_DIR})
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
