if(CODYCO_BUILD_TESTS)
  enable_testing()
  add_subdirectory(iDynTreeTests)
  add_subdirectory(wholeBodyModelIcubTest)

endif()
  add_subdirectory(codycoCommonsTest)
add_subdirectory(wholeBodyInterfaceTest)
add_subdirectory(wholeBodyInterfaceIcubTest)
if(CODYCO_USES_URDFDOM)
    add_subdirectory(yarpWholeBodyInterfaceTest)
endif()
add_subdirectory(inverseDynamicsTest)

