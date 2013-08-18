
# Copyright (C) 2010 RobotCub Consortium
# Author:  Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME iDynTreeJacobianTest)

PROJECT(${PROJECTNAME})

# YARP needs CMake 2.6 or greater
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

# find YARP
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(Orocos-KDL REQUIRED)
find_package(kdl_codyco REQUIRED)
find_package(Eigen3 REQUIRED)


# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${kdl_codyco_INCLUDE_DIRS})
 
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${iDyn_INCLUDE_DIRS}
					${iDynTree_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${skinDynLib_INCLUDE_DIRS})
                    
message("idyntree ${iDynTree_LIBRARIES}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall ${ICUB_LINK_FLAGS}")

add_executable(${PROJECTNAME} iDynTreeJacobianTest.cpp)

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} iDyn iDynTree ${iDynTree_LIBRARIES} ${Orocos-KDL_LIBRARIES} ${kdl_codyco_LIBRARIES})
