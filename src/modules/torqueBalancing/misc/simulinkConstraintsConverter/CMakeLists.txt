add_library(simulinkConstraintsConverter MODULE SimulinkConstraintsConverter.h SimulinkConstraintsConverter.cpp)
target_link_libraries(simulinkConstraintsConverter ${YARP_LIBRARIES})

if (APPLE)
    set_target_properties(simulinkConstraintsConverter PROPERTIES SUFFIX ".dylib")
endif()

install(TARGETS simulinkConstraintsConverter DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)