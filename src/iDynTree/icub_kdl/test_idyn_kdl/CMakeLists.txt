
# Copyright (C) 2010 RobotCub Consortium
# Author: Matteo Fumagalli, Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME misc_src)

PROJECT(${PROJECTNAME})



# YARP needs CMake 2.6 or greater
cmake_minimum_required(VERSION 2.8)


set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find YARP
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Orocos-KDL REQUIRED)
find_package(kdl_urdf REQUIRED)
find_package(kdl_codyco REQUIRED)


# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${kdl_codyco_INCLUDE_DIRS})
 

#FILE(GLOB folder_source force_test.cpp ../lib/chain_conversion.cpp ../lin/inertia_common.cpp)
#FILE(GLOB folder_header ../lib/chain_conversion.h ../lib/inertia_common.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${iDyn_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})
                    
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")


#ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

#TARGET_LINK_LIBRARIES(${PROJECTNAME} iDyn ${YARP_LIBRARIES} skinDynLib ${Orocos-KDL_LIBRARIES})

#INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_executable(force_test_icub_arm force_test_icub_arm.cpp iDyn_KDL_emulation.cpp iDyn_KDL_emulation.h ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h ../iDyn_KDL_conversion/KDL2iDyn.cpp ../iDyn_KDL_conversion/KDL2iDyn.h ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)
#add_executable(force_test_no_sensor force_test_no_sensor.cpp ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h  ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)
#add_executable(force_test_3links force_test_3links.cpp ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h  ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)
#add_executable(force_test_5links force_test_5links.cpp ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h  ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)
add_executable(force_test_7links force_test_7links.cpp iDyn_KDL_emulation.cpp iDyn_KDL_emulation.h ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h ../iDyn_KDL_conversion/KDL2iDyn.cpp ../iDyn_KDL_conversion/KDL2iDyn.h ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)
add_executable(force_test_execution_time force_test_execution_time.cpp ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h  ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp)
add_executable(iCubWholeBodyCOM_kdl_idyn iCubWholeBodyCOM_kdl_idyn.cpp iDyn_KDL_emulation.cpp iDyn_KDL_emulation.h ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h ../iDyn_KDL_conversion/KDL2iDyn.cpp ../iDyn_KDL_conversion/KDL2iDyn.h ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp ../icub_model/idyn2kdl_icub.h ../icub_model/idyn2kdl_icub.cpp)
add_executable(iCubWholeBodyInvDyn_kdl_urdf iCubWholeBodyInvDyn_kdl_urdf.cpp iDyn_KDL_emulation.cpp iDyn_KDL_emulation.h ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h ../iDyn_KDL_conversion/KDL2iDyn.cpp ../iDyn_KDL_conversion/KDL2iDyn.h ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp ../icub_model/idyn2kdl_icub.h ../icub_model/idyn2kdl_icub.cpp)
add_executable(iCubWholeBodyFwdPos_kdl_urdf iCubWholeBodyFwdPos_kdl_urdf.cpp iDyn_KDL_emulation.cpp iDyn_KDL_emulation.h ../iDyn_KDL_conversion/iDyn2KDL.cpp ../iDyn_KDL_conversion/iDyn2KDL.h ../iDyn_KDL_conversion/KDL2iDyn.cpp ../iDyn_KDL_conversion/KDL2iDyn.h ./custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.cpp ../icub_model/idyn2kdl_icub.h ../icub_model/idyn2kdl_icub.cpp)
add_executable(check_kdl_export check_kdl_export.cpp)



target_link_libraries(force_test_icub_arm ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})
#target_link_libraries(force_test_no_sensor ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})
#target_link_libraries(force_test_3links ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})
#target_link_libraries(force_test_5links ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})
target_link_libraries(force_test_7links ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})
target_link_libraries(force_test_execution_time ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES})
target_link_libraries(iCubWholeBodyCOM_kdl_idyn ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES} ${kdl_codyco_LIBRARIES} ${kdl_urdf_LIBRARIES})
target_link_libraries(iCubWholeBodyInvDyn_kdl_urdf ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES} ${kdl_codyco_LIBRARIES} ${kdl_urdf_LIBRARIES})
target_link_libraries(iCubWholeBodyFwdPos_kdl_urdf ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES} ${kdl_codyco_LIBRARIES} ${kdl_urdf_LIBRARIES})
target_link_libraries(check_kdl_export ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${Orocos-KDL_LIBRARIES} ${kdl_codyco_LIBRARIES} ${kdl_urdf_LIBRARIES})

