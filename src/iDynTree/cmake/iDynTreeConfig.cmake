if (iDynTree_CONFIG_INCLUDED)
  return()
endif()
set(iDynTree_CONFIG_INCLUDED TRUE)

set(iDynTree_INCLUDE_DIRS /home/icub/software/src/iCub/main/build/include)

foreach(lib iDynTree)
  set(onelib "${lib}-NOTFOUND")
  find_library(onelib ${lib}
    PATHS /home/icub/software/src/iCub/main/build/lib
    NO_DEFAULT_PATH
    )
  if(NOT onelib)
    message(FATAL_ERROR "Library '${lib}' in package iDynTree is not installed properly")
  endif()
  list(APPEND iDynTree_LIBRARIES ${onelib})
endforeach()

foreach(dep ICUB)
  if(NOT ${dep}_FOUND)
    find_package(${dep})
  endif()
  list(APPEND iDynTree_INCLUDE_DIRS ${${dep}_INCLUDE_DIRS})
  list(APPEND iDynTree_LIBRARIES ${${dep}_LIBRARIES})
endforeach()
