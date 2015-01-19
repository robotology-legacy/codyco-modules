# Copyright: (C) 2014
# Authors: ...
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

file(GLOB conf conf/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install( FILES ${conf} 
              DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${PROJECTNAME})   # directory name matches what is provided as "default context" in code

yarp_install( FILES ${scripts}
              DESTINATION ${CODYCO_APPLICATIONS_TEMPLATES_INSTALL_DIR})

