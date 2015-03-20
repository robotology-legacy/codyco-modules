set(modulename steppingDemo)
set(appname    steppingDemo)

set(lua_fsm_step       ${CMAKE_CURRENT_SOURCE_DIR}/lua/fsm_step.lua)
set(lua_fsm_stepping   ${CMAKE_CURRENT_SOURCE_DIR}/lua/fsm_stepping.lua)
set(lua_bin            ${CMAKE_CURRENT_SOURCE_DIR}/lua/steppingDemo.lua)

yarp_install(PROGRAMS  ${lua_fsm} DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${modulename}/lua)

#workaround copied from interactiveObjectsLearning app in icub-contrib
# We cannot go for "yarp_install(PROGRAMS ${lua_bin} DESTINATION bin)", since on Windows, only
# at build time (i.e. not during installation), we have to handle the Debug|Release subfolders.
# Requirements: the cmake "-E copy" command does not change file permissions, hence the file needs
# to be stored in the versioning database with *executable* rights.
add_custom_target(copy_lua_in_build ALL)
add_custom_command(TARGET copy_lua_in_build POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${lua_bin} ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
                   COMMENT "Copying ${lua_bin} to ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}/")
install(PROGRAMS ${lua_bin} DESTINATION bin)


add_subdirectory(app)
