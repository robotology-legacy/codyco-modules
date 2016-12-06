add_subdirectory(wholeBodyDynamics3)
add_subdirectory(torqueGraspManager)
add_subdirectory(eventRepeater)
add_subdirectory(torqueBalancing)
add_subdirectory(torqueBalancingReferencesGenerator)
add_subdirectory(reachRandomJointPositions)

if(CODYCO_USES_KDL)
    add_subdirectory(wholeBodyDynamicsTree)
    add_subdirectory(icubWalkingIK)

    if(CODYCO_USES_OROCOS_BFL_BERDY)
        add_subdirectory(wholeBodyEstimator)
    endif()
endif()

add_subdirectory(walkPlayer)

if(NOT MSVC)
    add_subdirectory(codycoTrajGenDemoY2)
endif()
