set(robots iCubGenova01
           iCubGenova03
           icubGazeboSim
           iCubParis01
           iCubParis02)

foreach(robot ${robots})
    yarp_install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/robots/${robot}
                 DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR})
endforeach()
