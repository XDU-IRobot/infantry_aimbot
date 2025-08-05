
add_library(daheng_cam SHARED
  ${CMAKE_CURRENT_LIST_DIR}/src/daheng.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/daheng_ext_trig.cc
)

target_include_directories(daheng_cam PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include/daheng_cam)
target_include_directories(daheng_cam PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

find_package(OpenCV REQUIRED)

target_link_libraries(daheng_cam PRIVATE
  gxiapi
)

target_link_libraries(daheng_cam PUBLIC
  ${OpenCV_LIBS}
)
