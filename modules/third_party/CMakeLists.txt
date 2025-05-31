
add_subdirectory(backward-cpp)

find_library(BACKWARD_DW_LIBRARY NAMES dw PATHS /usr/lib /usr/lib64)
if(BACKWARD_DW_LIBRARY)
  message(STATUS "Found libdw: ${BACKWARD_DW_LIBRARY}")
  add_definitions(-DBACKWARD_HAS_DW=1) # apt install libdw-dev
else()
  message(STATUS "libdw not found, disabling DW support in backward-cpp, no backtrace will be available.")
  message(STATUS "HINT---Install it with: sudo apt install libdw-dev")
endif()

add_subdirectory(toml11)

add_library(third_party_modules INTERFACE)

target_link_libraries(third_party_modules
  INTERFACE
  Backward::Interface
  toml11::toml11
)
