
set(BUILD_TESTING OFF)
set(TBB_TEST OFF)

add_subdirectory(daheng_cam)
add_subdirectory(backward-cpp)
add_subdirectory(outcome)
add_subdirectory(pfr)
add_subdirectory(oneTBB)
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