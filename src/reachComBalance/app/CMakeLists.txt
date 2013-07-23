set(appname reachComBalance)
set(yscopename yarpscope)

icub_app(${appname})

SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")

MESSAGE(STATUS ${ICUB_APPLICATIONS_PREFIX})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

file(GLOB yscope ${CMAKE_CURRENT_SOURCE_DIR}/conf/yarpscope_ISIRcontroller.xml)

icub_app_install(${appname}
        FILES ${conf}
        DESTINATION conf)

icub_app_install(${appname} 
                 FILES ${scripts}
                 DESTINATION scripts)

icub_app_install(${appname}
        FILES ${yscope}
        DESTINATION ../yarpscope/conf/reachComBalance)