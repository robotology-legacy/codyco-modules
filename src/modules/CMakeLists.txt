add_subdirectory(wholeBodyDynamicsTree)
add_subdirectory(wholeBodyGaze)
add_subdirectory(adaptiveControl)
#add_subdirectory(staticInertiaIdentification)
add_subdirectory(insituFTSensorCalibration)

   add_subdirectory(torqueBalancing)
if(${CODYCO_USES_EIGEN_320})
   add_subdirectory(locomotionControl)
   add_subdirectory(locomotionPlanner)
   add_subdirectory(torqueBalancing)
   #add_subdirectory(motorFrictionExcitation)
   #add_subdirectory(motorFrictionIdentification)
   add_subdirectory(wholeBodyReach)
endif()

add_subdirectory(walkPlayer)

add_subdirectory(codycoCoordinatorDemo1Y)

#add_subdirectory(wholeBodyNeckVelocity)
