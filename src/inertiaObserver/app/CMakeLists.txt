########## application
set(appname inertiaObserver)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
                  
file(GLOB data ${CMAKE_CURRENT_SOURCE_DIR}/data/*)

icub_app_install(${appname}
		FILES ${conf}
		DESTINATION conf)

icub_app_install(${appname} 
                 FILES ${scripts}
                 DESTINATION scripts)
                 
icub_app_install(${appname} 
                 FILES ${data}
                 DESTINATION data)




icub_app_all()
