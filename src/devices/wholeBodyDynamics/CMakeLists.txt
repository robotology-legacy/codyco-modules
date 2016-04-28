# Copyright: (C) 2016 Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the GNU LGPL v2+

find_package(YARP REQUIRED)
find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${YARP_MODULE_PATH}")
include(YarpInstallationHelpers)

yarp_configure_external_installation(codyco)

yarp_prepare_device(wholeBodyDynamicsDevice TYPE WholeBodyDynamicsDevice INCLUDE WholeBodyDynamicsDevice.h)

set(ENABLE_codycomod_wholeBodyDynamics TRUE)

if(NOT SKIP_wholeBodyDynamics)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(SYSTEM
                    ${YARP_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

yarp_add_plugin(wholeBodyDynamicsDevice WholeBodyDynamicsDevice.h WholeBodyDynamicsDevice.cpp)
target_link_libraries(wholeBodyDynamicsDevice wholeBodyDynamicsSettings wholeBodyDynamics_IDLServer ${YARP_LIBRARIES})
yarp_install(TARGETS wholeBodyDynamicsDevice
             EXPORT CoDyCo
             COMPONENT runtime
             LIBRARY DESTINATION ${CODYCO_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${CODYCO_STATIC_PLUGINS_INSTALL_DIR})

endif()