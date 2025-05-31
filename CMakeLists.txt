cmake_minimum_required(VERSION 3.8)
project(infantry_aimbot)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(cmake/CheckCppStd.cmake)

# mold installed?
# 用mold链接比较快，建议sudo apt install mold
find_program(MOLD_EXECUTABLE NAMES mold PATHS /usr/bin /usr/local/bin)
if(MOLD_EXECUTABLE)
  message(STATUS "Using mold linker: ${MOLD_EXECUTABLE}")
  set(CMAKE_EXE_LINKER_FLAGS "-fuse-ld=mold")
  set(CMAKE_SHARED_LINKER_FLAGS "-fuse-ld=mold")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
  message(STATUS "No build type specified, defaulting to Release")
endif()

add_definitions(-Wall -Wextra -Wpedantic)

add_subdirectory(modules)
file(GLOB_RECURSE SOURCES "src/*.cc")
add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_modules)

# 设置好CONFIG_DIR宏
target_compile_definitions(${PROJECT_NAME} PRIVATE CONFIG_DIR="${CMAKE_CURRENT_LIST_DIR}/config")

# 打印依赖关系图
include(cmake/DependencyGraph.cmake)
message("\n==================== DEPENDENCY GRAPH ====================")
message("${PROJECT_NAME}")
print_dependency_graph(${PROJECT_NAME} "")
message("==========================================================\n")
