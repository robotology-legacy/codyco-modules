add_subdirectory(libraries)
add_subdirectory(idl)
add_subdirectory(devices)

if(${CODYCO_USES_KDL})
    add_subdirectory(modules)
    add_subdirectory(scripts)
endif()
