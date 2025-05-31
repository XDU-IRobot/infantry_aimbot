
add_library(param_manager SHARED
  src/param_manager.cc
)
target_include_directories(param_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(param_manager PRIVATE third_party_modules)