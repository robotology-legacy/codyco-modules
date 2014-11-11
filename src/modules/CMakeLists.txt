add_subdirectory(wholeBodyDynamicsTree)
add_subdirectory(wholeBodyGaze)
add_subdirectory(adaptiveControl)
#add_subdirectory(staticInertiaIdentification)
if(${CODYCO_USES_EIGEN_320})
#add_subdirectory(locomotionControl)
add_subdirectory(locomotionPlanner)
add_subdirectory(torqueBalancing)
add_subdirectory(motorFrictionExcitation)
add_subdirectory(motorFrictionIdentification)
endif()

#add_subdirectory(jointTorqueControl)


add_subdirectory(codycoCoordinatorDemo1Y)
if(CODYCO_BUILD_WHOLEBODYREACH)
    add_subdirectory(wholeBodyReach)
endif()
