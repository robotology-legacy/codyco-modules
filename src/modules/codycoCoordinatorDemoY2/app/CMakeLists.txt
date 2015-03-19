set(modulename ${PROJECT_NAME})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/*.xml)
#
yarp_install( FILES ${conf}
              DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${modulename})

yarp_install( FILES ${scripts}
              DESTINATION ${CODYCO_APPLICATIONS_TEMPLATES_INSTALL_DIR})

