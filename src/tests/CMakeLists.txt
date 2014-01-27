if(CODYCO_BUILD_TESTS)
  enable_testing()
  add_subdirectory(iDynTreeTests)
endif()
add_subdirectory(wholeBodyInterfaceTest)
add_subdirectory(wholeBodyInterfaceIcubTest)

