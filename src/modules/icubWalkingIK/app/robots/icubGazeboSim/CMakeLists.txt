set(robotname icubGazeboSim)

file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)
file(GLOB csv ${CMAKE_CURRENT_SOURCE_DIR}/csv/*.csv)

yarp_install(FILES ${ini} DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR}/${robotname})
yarp_install(FILES ${csv} DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR}/${robotname})
