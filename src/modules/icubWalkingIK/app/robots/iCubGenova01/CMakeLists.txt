set(robotname iCubGenova01)
configure_file(iCubWalkingIKModule.ini.in ${CMAKE_CURRENT_SOURCE_DIR}/iCubWalkingIKModule.ini)
<<<<<<< a441e415eb4af2e84c49d16ed99e3cb8ea344968
=======
message(CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR})
>>>>>>> minor edits on impedance walk

file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

yarp_install(FILES ${ini} DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR}/${robotname})
yarp_install(FILES ${csv} DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR}/${robotname})
