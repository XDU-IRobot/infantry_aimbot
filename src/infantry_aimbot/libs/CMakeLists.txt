
set(BUILD_TESTING OFF)

add_subdirectory(daheng_cam)
add_subdirectory(backward-cpp)
add_subdirectory(outcome)
add_subdirectory(pfr)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(OpenMP REQUIRED)

unset(BUILD_TESTING)

add_library(${PROJECT_NAME}_libs INTERFACE)

target_link_libraries(${PROJECT_NAME}_libs INTERFACE
  daheng_cam
  ${OpenCV_LIBS}
  Backward::Interface
  outcome
  Eigen3::Eigen
  Boost::pfr
  TBB::tbb
  # OpenMP::OpenMP_CXX
)

target_include_directories(${PROJECT_NAME}_libs INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/kalman/include
)