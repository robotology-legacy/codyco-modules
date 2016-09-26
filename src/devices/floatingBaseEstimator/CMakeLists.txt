# Copyright: (C) 2016 Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the GNU LGPL v2+

find_package(YARP REQUIRED)
find_package(iDynTree REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${YARP_MODULE_PATH}")
include(YarpInstallationHelpers)

yarp_configure_external_installation(codyco)

yarp_prepare_plugin(floatingbaseestimator CATEGORY device
                                          TYPE yarp::dev::floatingBaseEstimator
                                          INCLUDE "floatingBaseEstimator.h"
                                          DEFAULT ON
                                          ADVANCED)

if(ENABLE_codycomod_floatingbaseestimator)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    include_directories(SYSTEM
                        ${YARP_INCLUDE_DIRS}
                        ${EIGEN3_INCLUDE_DIR})

    yarp_add_plugin(floatingBaseEstimator floatingBaseEstimator.h floatingBaseEstimator.cpp)

    target_link_libraries(floatingBaseEstimator floatingBaseEstimatorRPC
                                                ctrlLibRT
                                                ${YARP_LIBRARIES}
                                                ${iDynTree_LIBRARIES})

    if(MSVC)
        add_definitions(-D_USE_MATH_DEFINES)
    endif()


    yarp_install(TARGETS floatingBaseEstimator
                 EXPORT CoDyCo
                 COMPONENT runtime
                 LIBRARY DESTINATION ${CODYCO_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${CODYCO_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES floatingbaseestimator.ini DESTINATION ${CODYCO_PLUGIN_MANIFESTS_INSTALL_DIR})
endif()
