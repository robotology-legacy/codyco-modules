# Copyright: (C) 2016 Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the GNU LGPL v2+

find_package(YARP REQUIRED)
# ICUB is required for skinDynLib
find_package(ICUB REQUIRED)
find_package(iDynTree REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${YARP_MODULE_PATH}")
include(YarpInstallationHelpers)

yarp_configure_external_installation(codyco)

yarp_prepare_device(wholebodydynamics TYPE WholeBodyDynamicsDevice INCLUDE "WholeBodyDynamicsDevice.h")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(SYSTEM
                    ${YARP_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${skinDynLib_INCLUDE_DIRS})

yarp_add_plugin(wholeBodyDynamicsDevice WholeBodyDynamicsDevice.h WholeBodyDynamicsDevice.cpp
                                        SixAxisForceTorqueMeasureHelpers.h SixAxisForceTorqueMeasureHelpers.cpp
                                        GravityCompensationHelpers.h GravityCompensationHelpers.cpp)
                                        
target_link_libraries(wholeBodyDynamicsDevice wholeBodyDynamicsSettings
                                              wholeBodyDynamics_IDLServer
                                              ctrlLibRT
                                              ${YARP_LIBRARIES}
                                              skinDynLib
                                              ${iDynTree_LIBRARIES})

if(MSVC)
   add_definitions(-D_USE_MATH_DEFINES)
endif()


yarp_install(TARGETS wholeBodyDynamicsDevice
             EXPORT CoDyCo
             COMPONENT runtime
             LIBRARY DESTINATION ${CODYCO_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${CODYCO_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES wholebodydynamics.ini DESTINATION ${CODYCO_PLUGIN_MANIFESTS_INSTALL_DIR})
