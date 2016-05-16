# Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Jorhabib Eljaik jorhabib.eljaik@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(modulename wholeBodyNeckVelocity)


file(GLOB app ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

yarp_install(FILES ${app}      DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${modulename})
