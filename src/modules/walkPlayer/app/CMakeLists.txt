# Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Marco Randazzo - marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(modulename walkPlayer)


# SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

file(GLOB sequences ${CMAKE_CURRENT_SOURCE_DIR}/sequences/*.txt)

yarp_install(FILES ${conf}      DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${modulename})
yarp_install(FILES ${scripts}   DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${modulename})
yarp_install(FILES ${sequences} DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${modulename})

# icub_app_install(${appname}
#         FILES ${conf}
#         DESTINATION conf)
#
# icub_app_install(${appname}
#         FILES ${sequences}
#         DESTINATION sequences)
#
# icub_app_install(${appname}
#                  FILES ${scripts}
#                  DESTINATION scripts)

