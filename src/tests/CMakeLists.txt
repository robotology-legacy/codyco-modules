if(CODYCO_BUILD_TESTS)
  enable_testing()
  add_subdirectory(iDynTreeTests)
  add_subdirectory(wholeBodyModelIcubTest)
endif()
add_subdirectory(wholeBodyInterfaceTest)
add_subdirectory(wholeBodyInterfaceIcubTest)
add_subdirectory(inverseDynamicsTest)

