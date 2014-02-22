
enable_testing()

include_directories(${EIGEN3_INCLUDE_DIR} ${wholeBodyInterface_INCLUDE_DIRS}  ${wholeBodyInterfaceIcub_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS})

add_executable(wbmIcubInvDynTest wbmIcubInverseDynamicsTest.cpp)

target_link_libraries(wbmIcubInvDynTest ${ICUB_LIBRARIES} ${orocos_kdl_LIBRARIES} wholeBodyInterface wholeBodyInterfaceIcub kdl_codyco )

add_test(NAME test_wbmIcubInvDynTest COMMAND wbmIcubInvDynTest)
