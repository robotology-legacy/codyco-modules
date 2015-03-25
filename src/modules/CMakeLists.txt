add_subdirectory(wholeBodyDynamicsTree)
add_subdirectory(wholeBodyGaze)
add_subdirectory(adaptiveControl)
#add_subdirectory(staticInertiaIdentification)
add_subdirectory(insituFTSensorCalibration)
add_subdirectory(codycoDemoY2/app)
add_subdirectory(RedBallGraspDemo)
add_subdirectory(eventRepeater)

if(${CODYCO_USES_EIGEN_320})
   add_subdirectory(locomotionControl)
   add_subdirectory(locomotionPlanner)
   add_subdirectory(torqueBalancing)
   #add_subdirectory(motorFrictionExcitation)
   #add_subdirectory(motorFrictionIdentification)
   add_subdirectory(wholeBodyReach)
endif()

add_subdirectory(walkPlayer)

add_subdirectory(codycoCoordinatorDemoY1)
add_subdirectory(codycoCoordinatorDemoY2)

#add_subdirectory(wholeBodyNeckVelocity)
