cmake_minimum_required(VERSION 3.10)
project(outcome)

# file(DOWNLOAD
#     https://github.com/ned14/outcome/raw/master/single-header/outcome.hpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/include/outcome.hpp
# )

add_library(outcome INTERFACE)
target_include_directories(outcome INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
