if(CODYCO_USES_WBI_TOOLBOX)
    message(STATUS "Adding -fPIC option to compiler")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif(CODYCO_USES_WBI_TOOLBOX)

add_subdirectory(modules)
add_subdirectory(misc/ftDumper)

add_subdirectory(tests)


if (ICUB_USE_XDE)
     add_subdirectory(reachComBalance)
     add_subdirectory(tests/ISIRControllerYarpTest)
     message(STATUS "ISIR modules with XDE: YES")
else (ICUB_USE_XDE)
     message(STATUS "ISIR modules with XDE: NO")
endif (ICUB_USE_XDE)
