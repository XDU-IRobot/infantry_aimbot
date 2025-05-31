function(print_dependency_graph target indent)
  include(${CMAKE_HOME_DIRECTORY}/cmake/ColorPrint.cmake)

  get_target_property(interface_deps ${target} INTERFACE_LINK_LIBRARIES)
  if(NOT interface_deps)
    set(interface_deps "")
  endif()
  get_target_property(link_deps ${target} LINK_LIBRARIES)
  if(NOT link_deps)
    set(link_deps "")
  endif()

  set(all_deps ${interface_deps} ${link_deps})
  list(REMOVE_DUPLICATES all_deps)

  foreach(dep IN LISTS all_deps)
    if(TARGET ${dep})
      # 判断依赖类型
      list(FIND interface_deps ${dep} idx_interface)
      list(FIND link_deps ${dep} idx_link)

      if(idx_interface GREATER -1 AND idx_link GREATER -1)
        set(color "${ColourBoldGreen}") # PUBLIC
        set(should_recurse TRUE)
      elseif(idx_interface GREATER -1)
        set(color "${ColourReset}") # INTERFACE
        set(should_recurse TRUE)
      elseif(idx_link GREATER -1)
        set(color "${ColourBlue}") # PRIVATE
        set(should_recurse FALSE)
      else()
        set(color "${ColourReset}")
        set(should_recurse TRUE)
      endif()

      message("${indent}  └─ ${color}${dep}${ColourReset}")
      if(should_recurse)
        print_dependency_graph(${dep} "   ${indent}")
      endif()
    endif()
  endforeach()
endfunction()
