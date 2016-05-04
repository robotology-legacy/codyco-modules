<<<<<<< a441e415eb4af2e84c49d16ed99e3cb8ea344968
add_subdirectory(libraries)
add_subdirectory(idl)
=======
if(CODYCO_USES_WBI_TOOLBOX)
    message(STATUS "Adding -fPIC option to compiler")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif(CODYCO_USES_WBI_TOOLBOX)

>>>>>>> minor edits on impedance walk
add_subdirectory(devices)

if(${CODYCO_USES_KDL})
    add_subdirectory(modules)
    add_subdirectory(misc/ftDumper)
    add_subdirectory(misc/reachRandomJointPositions)
    #add_subdirectory(misc/skinContactGenerator)
    add_subdirectory(scripts)
    add_subdirectory(tests)
endif()
