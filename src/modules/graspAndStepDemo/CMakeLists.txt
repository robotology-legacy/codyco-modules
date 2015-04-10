set(modulename graspAndStepDemo)
set(appname    graspAndStepDemo)

set(lua_fsm             ${CMAKE_CURRENT_SOURCE_DIR}/lua/fsm_graspAndStep.lua
                        ${CMAKE_CURRENT_SOURCE_DIR}/lua/fsm_grasping.lua
                        ${CMAKE_CURRENT_SOURCE_DIR}/lua/fsm_left_step.lua
                        ${CMAKE_CURRENT_SOURCE_DIR}/lua/fsm_right_step.lua
                        ${CMAKE_CURRENT_SOURCE_DIR}/lua/fsm_left_right_sway.lua
                        ${CMAKE_CURRENT_SOURCE_DIR}/lua/fsm_ping_pong.lua
                        ${CMAKE_CURRENT_SOURCE_DIR}/lua/gas_funcs.lua)
set(lua_bin             ${CMAKE_CURRENT_SOURCE_DIR}/lua/pingPongDemo.lua
                        ${CMAKE_CURRENT_SOURCE_DIR}/lua/leftRightBalancingDemo.lua
                        ${CMAKE_CURRENT_SOURCE_DIR}/lua/graspAndStepDemo.lua)

yarp_install(PROGRAMS  ${lua_fsm} DESTINATION ${CODYCO_CONTEXTS_INSTALL_DIR}/${modulename}/lua)

#workaround copied from interactiveObjectsLearning app in icub-contrib
# We cannot go for "yarp_install(PROGRAMS ${lua_bin} DESTINATION bin)", since on Windows, only
# at build time (i.e. not during installation), we have to handle the Debug|Release subfolders.
# Requirements: the cmake "-E copy" command does not change file permissions, hence the file needs
# to be stored in the versioning database with *executable* rights.

set(LLVAR 0)
foreach(lua_component IN LISTS lua_bin)
    set(lua_component_target "gas_copy_lua_in_build${LLVAR}")
    add_custom_target(${lua_component_target} ALL)
    add_custom_command(TARGET ${lua_component_target} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${lua_component} ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
                   COMMENT "Copying ${lua_component} to ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}/")
    install(PROGRAMS ${lua_component} DESTINATION bin)
    MATH(EXPR LLVAR "${LLVAR}+1")
endforeach()

add_subdirectory(app)

add_subdirectory(portmonitor)