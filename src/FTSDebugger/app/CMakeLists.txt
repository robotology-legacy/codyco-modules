set(appname FTSDebugger)

icub_app(${appname})

SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")

MESSAGE(STATUS ${ICUB_APPLICATIONS_PREFIX})

file(GLOB yscope ${CMAKE_CURRENT_SOURCE_DIR}/conf/yarpscope/*.xml)

icub_app_install(${appname} 
                 FILES ${yscope}
                 DESTINATION ../yarpscope/conf/FTSDebugger)

