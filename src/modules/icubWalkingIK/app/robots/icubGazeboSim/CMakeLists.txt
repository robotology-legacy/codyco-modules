set(robotname icubGazeboSim)
configure_file(iCubWalkingIKModule.ini.in ${CMAKE_CURRENT_SOURCE_DIR}/iCubWalkingIKModule.ini)
message(CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

yarp_install(FILES ${ini} DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR}/${robotname})
yarp_install(FILES ${csv} DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR}/${robotname})
