# Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Marco Randazzo - marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname walkPlayer)

icub_app(${appname})

SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

file(GLOB sequences ${CMAKE_CURRENT_SOURCE_DIR}/sequences/*.txt)

icub_app_install(${appname}
        FILES ${conf}
        DESTINATION conf)

icub_app_install(${appname}
        FILES ${sequences}
        DESTINATION sequences)

icub_app_install(${appname} 
                 FILES ${scripts}
                 DESTINATION scripts)

