if(SIMULINK_LIBRARY)
    message(STATUS "Adding -fPIC option to compiler")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif(SIMULINK_LIBRARY)

add_subdirectory(tests)
add_subdirectory(libraries)
#add_subdirectory(sensorsInterfaces)
#add_subdirectory(wbci)
#add_subdirectory(wbciy)
add_subdirectory(modules)


if (CODYCO_DASHBOARD_SUBMIT OR CODYCO_BUILD_TESTS)
   add_subdirectory(tests)
endif()

#built ftDumper only if YARP24 is found
if(CODYCO_BUILD_YARP24_MODULES)
   add_subdirectory(misc/ftDumper)
endif(CODYCO_BUILD_YARP24_MODULES)

if (ICUB_USE_XDE)
     add_subdirectory(reachComBalance)
     add_subdirectory(tests/ISIRControllerYarpTest)
     message(STATUS "ISIR modules with XDE: YES")
else (ICUB_USE_XDE)
     message(STATUS "ISIR modules with XDE: NO")
endif (ICUB_USE_XDE)

if(SIMULINK_LIBRARY)
    add_subdirectory(simulink)
    message(STATUS "Simulink library will be compiled")
else(SIMULINK_LIBRARY)
    message(STATUS "Simulink library skipped")
endif(SIMULINK_LIBRARY)
