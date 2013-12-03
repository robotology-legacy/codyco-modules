set(appname comStepper)
set(yscopename yarpscope)

icub_app(${appname})

SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")

MESSAGE(STATUS ${ICUB_APPLICATIONS_PREFIX})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

file(GLOB yscope ${CMAKE_CURRENT_SOURCE_DIR}/conf/yarpscope_comStepperErrors.xml
                 ${CMAKE_CURRENT_SOURCE_DIR}/conf/yarpscope_comStepperControls.xml)

yarp_install( FILES ${conf}
              DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${appname})

yarp_install( FILES ${scripts}
              DESTINATION ${CODYCO_APPLICATIONS_TEMPLATES_INSTALL_DIR})

yarp_install( FILES ${yscope}
        DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${appname}/yarpscope)
