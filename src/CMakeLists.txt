add_subdirectory(libraries) 
add_subdirectory(wbi)
add_subdirectory(wbiy)
add_subdirectory(sensorsInterfaces)
#add_subdirectory(wbci)
#add_subdirectory(wbciy)

add_subdirectory(modules/comStepper)

add_subdirectory(locomotionControl)
add_subdirectory(locomotionPlanner)

add_subdirectory(tests)
#add_subdirectory(wholeBodyInterfaceYarpTest)


if (ICUB_USE_XDE)
     add_subdirectory(reachComBalance)
     add_subdirectory(tests/ISIRControllerYarpTest)
     message(STATUS "ISIR modules with XDE: YES")
else (ICUB_USE_XDE)
     message(STATUS "ISIR modules with XDE: NO")
endif (ICUB_USE_XDE)


