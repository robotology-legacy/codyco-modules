# Copyright (C) 2010 RobotCub Consortium
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

 
SET(PROJECTNAME icub_urdf)

PROJECT(${PROJECTNAME})

# YARP needs CMake 2.6 or greater
cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find YARP
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Orocos-KDL REQUIRED)
find_package(urdfdom REQUIRED)
find_package(kdl_urdf REQUIRED)
find_package(TinyXML REQUIRED)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIR} ${kdl_urdf_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


#FILE(GLOB folder_source normal_excited_regressor_test.cpp ../lib/inertia_common.cpp)
#FILE(GLOB folder_header ../lib/inertia_common.h)

#SOURCE_GROUP("Source Files" FILES ${folder_source})
#SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${iDyn_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS})

#ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

#TARGET_LINK_LIBRARIES(${PROJECTNAME} iDyn ${YARP_LIBRARIES} skinDynLib)

#INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_executable(icub_urdf_from_iDyn  icub_urdf_from_iDyn.cpp)
                                    
add_definitions("-lkdl_export")

target_link_libraries(icub_urdf_from_iDyn ${YARP_LIBRARIES} ${ICUB_LIBRARIES} iDynTree ${Orocos-KDL_LIBRARIES} ${urdfdom_LIBRARIES} ${kdl_urdf_LIBRARIES} ${TinyXML_LIBRARIES})
