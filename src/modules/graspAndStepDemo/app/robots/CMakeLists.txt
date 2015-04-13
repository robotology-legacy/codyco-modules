option(CODYCO_INSTALL_ALL_ROBOTS "Installation of all available robots" ON)

set(ROBOT_NAME "$ENV{YARP_ROBOT_NAME}" CACHE PATH
    "Name of your robot")

set(ROBOT_NAMES "icubGazeboSim"
                "iCubParis01"
                "iCubParis02")

macro(gas_install_robot_files robotname)
    file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/${robotname}/*.ini)
    yarp_install(FILES ${ini} DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR}/${robotname})
endmacro()

# If no env variable YARP_ROBOT_NAME is defined
if(CODYCO_INSTALL_ALL_ROBOTS)
    foreach(robotname IN LISTS ROBOT_NAMES)
        gas_install_robot_files(${robotname})
    endforeach()
else()
    if(ROBOT_NAME)
        gas_install_robot_files(${ROBOT_NAME})
    endif()
endif()