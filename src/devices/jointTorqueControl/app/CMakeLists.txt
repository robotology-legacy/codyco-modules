set(robots iCubGenova01
           iCubGenova03
	   icubGazeboSim
           iCubHeidelberg01)

foreach(robot ${robots})
    file(GLOB robot_conf ${CMAKE_CURRENT_SOURCE_DIR}/robots/${robot}/*.ini)
    yarp_install(FILES ${robot_conf}
                 DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR}/${robot})
endforeach()

