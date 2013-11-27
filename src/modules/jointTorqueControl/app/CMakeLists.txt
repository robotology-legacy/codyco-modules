set(appname jointTorqueControlModule)
set(yscopename yarpscope)

icub_app(${appname})

SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")

MESSAGE(STATUS ${ICUB_APPLICATIONS_PREFIX})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

icub_app_install(${appname}
        FILES ${conf}
        DESTINATION conf)

icub_app_install(${appname} 
                 FILES ${scripts}
                 DESTINATION scripts)

icub_app_install(${appname}
        FILES ${yscope}
        DESTINATION ../yarpscope/conf/jointTorqueControl)
