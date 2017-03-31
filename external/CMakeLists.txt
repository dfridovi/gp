# Google gtest for unit testing.
add_subdirectory(gtest)
message("gtest include dir: ${gtest_SOURCE_DIR}")
include_directories(${gtest_SOURCE_DIR})

# MatPlot++ plotting library.
add_subdirectory(matplotpp)
message("matplotpp include dir: ${matplotpp_SOURCE_DIR}")
include_directories(${matplotpp_SOURCE_DIR})
