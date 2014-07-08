

enable_testing()

find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS}
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${iDyn_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${iDynTree_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS})

add_executable(wbmIcubInvDynTest wbmIcubInverseDynamicsTest.cpp)

target_link_libraries(wbmIcubInvDynTest ${kdl_codyco_LIBRARIES}
                                        ${wholeBodyInterface_LIBRARIES}
                                        ${yarpWholeBodyInterface_LIBRARIES}
                                        ${YARP_LIBRARIES})

add_test(NAME test_wbmIcubInvDynTest COMMAND wbmIcubInvDynTest)
