
add_subdirectory(daheng_cam)

add_library(${PROJECT_NAME}_modules INTERFACE)

target_link_libraries(${PROJECT_NAME}_modules INTERFACE
  daheng_cam
)