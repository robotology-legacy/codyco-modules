set(modulename RedBallGrasp)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install( FILES ${conf}
              DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${modulename})

yarp_install( FILES ${scripts}
              DESTINATION ${CODYCO_APPLICATIONS_TEMPLATES_INSTALL_DIR})

