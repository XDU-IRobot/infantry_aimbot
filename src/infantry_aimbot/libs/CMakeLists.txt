
set(BUILD_TESTING OFF)

find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(daheng_cam)
add_subdirectory(backward-cpp)
add_subdirectory(outcome)
add_subdirectory(pfr)

# 海康相机驱动（可选，需要MVS SDK）
if(EXISTS "/opt/MVS/include/MvCameraControl.h")
  add_subdirectory(hikrobot_cam)
  set(HAS_HIKROBOT TRUE)
endif()

add_library(${PROJECT_NAME}_libs INTERFACE)

target_link_libraries(${PROJECT_NAME}_libs INTERFACE
  daheng_cam
  ${OpenCV_LIBS}
  Backward::Interface
  outcome
  Eigen3::Eigen
  Boost::pfr
  TBB::tbb
)

if(HAS_HIKROBOT)
  target_compile_definitions(${PROJECT_NAME}_libs INTERFACE HAS_HIKROBOT_CAM)
  target_link_libraries(${PROJECT_NAME}_libs INTERFACE hikrobot_cam MvCameraControl)
  target_include_directories(${PROJECT_NAME}_libs INTERFACE /opt/MVS/include)
  target_link_directories(${PROJECT_NAME}_libs INTERFACE /opt/MVS/lib/64)
endif()

target_include_directories(${PROJECT_NAME}_libs INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/kalman/include
)