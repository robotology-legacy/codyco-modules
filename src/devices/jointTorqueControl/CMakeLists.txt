# Copyright: (C) 2016 Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the GNU LGPL v2+

find_package(YARP REQUIRED)
# ICUB is required for skinDynLib
find_package(ICUB REQUIRED)
find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${YARP_MODULE_PATH}")
include(YarpInstallationHelpers)

yarp_configure_external_installation(codyco)

yarp_prepare_device(passThroughControlBoard TYPE yarp::dev::PassThroughControlBoard INCLUDE PassThroughControlBoard.h WRAPPER controlboardwrapper)
yarp_prepare_device(jointTorqueControl TYPE yarp::dev::JointTorqueControl INCLUDE JointTorqueControl.h WRAPPER controlboardwrapper)

set(ENABLE_codycomod_jointTorqueControl TRUE)

if(NOT SKIP_jointTorqueControl)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    include_directories(SYSTEM
                        ${EIGEN3_INCLUDE_DIR}
                        ${YARP_INCLUDE_DIRS}
                        ${EIGEN3_INCLUDE_DIR}
                        ${skinDynLib_INCLUDE_DIRS})

    yarp_add_plugin(jointTorqueControl JointTorqueControl.h JointTorqueControl.cpp PassThroughControlBoard.h  PassThroughControlBoard.cpp)
    target_link_libraries(jointTorqueControl ${YARP_LIBRARIES})

    yarp_add_plugin(passThroughControlBoard PassThroughControlBoard.h PassThroughControlBoard.cpp)
    target_link_libraries(passThroughControlBoard ${YARP_LIBRARIES})


    if(MSVC)
       add_definitions(-D_USE_MATH_DEFINES)
    endif()

    yarp_install(TARGETS passThroughControlBoard
                 EXPORT CoDyCo
                 COMPONENT runtime
                 LIBRARY DESTINATION ${CODYCO_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${CODYCO_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(TARGETS jointTorqueControl
                 EXPORT CoDyCo
                 COMPONENT runtime
                 LIBRARY DESTINATION ${CODYCO_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${CODYCO_STATIC_PLUGINS_INSTALL_DIR})

    add_subdirectory(app)
    
    yarp_install(FILES jointTorqueControl.ini DESTINATION ${CODYCO_PLUGIN_MANIFESTS_INSTALL_DIR})
    
endif()

