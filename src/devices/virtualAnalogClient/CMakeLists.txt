# Copyright: (C) 2016 Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the GNU LGPL v2+

find_package(YARP REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${YARP_MODULE_PATH}")
include(YarpInstallationHelpers)

yarp_configure_external_installation(codyco)

yarp_prepare_device(virtualAnalogClient TYPE VirtualAnalogClient INCLUDE "VirtualAnalogClient.h")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

include_directories(SYSTEM
                    ${YARP_INCLUDE_DIRS})

yarp_add_plugin(virtualAnalogClient VirtualAnalogClient.h VirtualAnalogClient.cpp)

target_link_libraries(virtualAnalogClient ${YARP_LIBRARIES})


yarp_install(TARGETS virtualAnalogClient
             EXPORT CoDyCo
             COMPONENT runtime
             LIBRARY DESTINATION ${CODYCO_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${CODYCO_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES virtualAnalogClient.ini DESTINATION ${CODYCO_PLUGIN_MANIFESTS_INSTALL_DIR})
