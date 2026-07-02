add_library(hikrobot_cam STATIC src/hikrobot_cam.cc)
target_include_directories(hikrobot_cam PUBLIC
  include
  ../daheng_cam/include
  /opt/MVS/include
  ${OpenCV_INCLUDE_DIRS})
target_link_directories(hikrobot_cam PUBLIC
  /opt/MVS/lib/64)
target_link_libraries(hikrobot_cam PUBLIC
  ${OpenCV_LIBS}
  MvCameraControl)
