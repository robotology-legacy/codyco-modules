enable_testing()

include_directories(${EIGEN3_INCLUDE_DIR}
                    ${orocos_kdl_INCLUDE_DIRS} 
                    ${iDynTree_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS} 
                    ${kdl_codyco_INCLUDE_DIRS}
                    ${iDyn_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS})

add_executable(iDynTreeJacobianTest iDynTreeJacobianTest.cpp)
#add_executable(iDynTreeWholeBodyFTsensorEstimateTest iDynTreeWholeBodyFTsensorEstimateTest.cpp)
add_executable(iDynTreeExternalWrenchesEstimationTest iDynTreeExternalWrenchesEstimationTest.cpp)

target_link_libraries(iDynTreeJacobianTest iDynTree ${orocos_kdl_LIBRARIES} ${kdl_codyco_LIBRARIES})
#target_link_libraries(iDynTreeWholeBodyFTsensorEstimateTest iDynTree ${orocos_kdl_LIBRARIES} ${kdl_codyco_LIBRARIES} )
target_link_libraries(iDynTreeExternalWrenchesEstimationTest iDynTree ${orocos_kdl_LIBRARIES} ${kdl_codyco_LIBRARIES}  )

add_test(NAME test_iDynTreeJacobianTest COMMAND iDynTreeJacobianTest)
#add_test(NAME test_iDynTreeWholeBodyFTsensorEstimateTest COMMAND iDynTreeWholeBodyFTsensorEstimateTest)
add_test(NAME test_iDynTreeExternalWrenchesEstimationTest COMMAND iDynTreeExternalWrenchesEstimationTest)
