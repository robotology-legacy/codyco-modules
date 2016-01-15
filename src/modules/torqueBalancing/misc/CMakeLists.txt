option(TORQUEBALANCING_BUILD_SIMULINK_CONSTRAINTS_CONVERTER "Build the portmonitor library to convert from simulink format to C++ format the constraints" NO)
mark_as_advanced(TORQUEBALANCING_BUILD_SIMULINK_CONSTRAINTS_CONVERTER)

if (TORQUEBALANCING_BUILD_SIMULINK_CONSTRAINTS_CONVERTER)
    add_subdirectory(simulinkConstraintsConverter)
endif()
