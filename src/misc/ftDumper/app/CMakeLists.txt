set(appname ftDumper)

#icub_app(${appname})
#SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")
#MESSAGE(STATUS ${ICUB_APPLICATIONS_PREFIX})

file(GLOB yscope ${CMAKE_CURRENT_SOURCE_DIR}/conf/yarpscope/*.xml)
file(GLOB yscopescripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)

yarp_install( FILES ${yscopescripts}
              DESTINATION ${CODYCO_APPLICATIONS_TEMPLATES_INSTALL_DIR})

yarp_install( FILES ${yscope}
        DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${appname}/yarpscope)


