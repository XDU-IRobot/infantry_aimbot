cmake_minimum_required(VERSION 3.8)
project(infantry_aimbot)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)
set(CMAKE_CXX_STANDARD 20)
add_definitions(-std=c++20)
set(CMAKE_CXX_STANDARD_REQUIRED true)

# # enable gprof for this node
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")

add_subdirectory(libs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 视频测试依赖
find_package(yaml-cpp REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

include_directories(include)

# 所有共享源文件（排除入口点）
file(GLOB_RECURSE SHARED_SOURCES src/*.cc)
list(FILTER SHARED_SOURCES EXCLUDE REGEX "src/main\\.cc$")
list(FILTER SHARED_SOURCES EXCLUDE REGEX "src/video_test\\.cc$")

# 主ROS节点
ament_auto_add_executable(${PROJECT_NAME}_node src/main.cc ${SHARED_SOURCES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_libs)
target_compile_definitions(${PROJECT_NAME}_node
  PRIVATE
  BACKWARD_HAS_DW=1
)

# 视频测试程序（使用ament_auto以获取ROS2头文件路径，但运行时不依赖ROS）
ament_auto_add_executable(video_test src/video_test.cc ${SHARED_SOURCES})
target_link_libraries(video_test ${PROJECT_NAME}_libs yaml-cpp)
target_compile_definitions(video_test
  PRIVATE
  BACKWARD_HAS_DW=1
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY assets
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
