set(appname motorFrictionExcitation)
set(yscopename yarpscope)

set(robots)

list(APPEND robots "iCubGenova01")
list(APPEND robots "iCubGenova02")
list(APPEND robots "iCubGenova03")
list(APPEND robots "iCubHeidelberg01")

foreach(robot IN robots)
    file(GLOB robot_conf ${CMAKE_CURRENT_SOURCE_DIR}/robots/${robot}/*.conf)
    yarp_install(FILES ${robot_conf}
                 DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR}/${robot})
endforeach()

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/*.xml)

file(GLOB yscope ${CMAKE_CURRENT_SOURCE_DIR}/debug_scope.xml)

yarp_install( FILES ${conf}
              DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${appname})

yarp_install( FILES ${scripts}
              DESTINATION ${CODYCO_APPLICATIONS_TEMPLATES_INSTALL_DIR})

yarp_install( FILES ${yscope}
        DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${appname}/yarpscope)
