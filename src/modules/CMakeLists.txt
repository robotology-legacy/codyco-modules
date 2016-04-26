add_subdirectory(wholeBodyDynamicsTree)
add_subdirectory(torqueGraspManager)
add_subdirectory(eventRepeater)
add_subdirectory(torqueBalancing)
add_subdirectory(torqueBalancingReferencesGenerator)

if(CODYCO_USES_OROCOS_BFL_BERDY)
    add_subdirectory(quaternionEKF)
endif()

add_subdirectory(wholeBodyEstimator)
add_subdirectory(icubWalkingIK)
add_subdirectory(walkPlayer)

add_subdirectory(codycoCoordinatorDemoY1)

if(NOT MSVC)
    add_subdirectory(codycoTrajGenDemoY2)
endif()
