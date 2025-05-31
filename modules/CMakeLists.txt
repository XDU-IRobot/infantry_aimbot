
# 注意顺序，模块之间有依赖关系
add_subdirectory(third_party)
add_subdirectory(param_manager)
add_subdirectory(daheng_cam)

add_library(${PROJECT_NAME}_modules INTERFACE)

target_link_libraries(${PROJECT_NAME}_modules INTERFACE
  daheng_cam
  param_manager
  third_party_modules
)
