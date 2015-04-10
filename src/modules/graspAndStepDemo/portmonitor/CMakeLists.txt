file(GLOB lua_portmonitor constraints_simulink.lua)

yarp_install( FILES ${lua_portmonitor}
              DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${modulename})