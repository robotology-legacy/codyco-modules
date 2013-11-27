# Copyright (C) 2013 CoDyCo
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME motorFrictionExcitation)
PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
find_package(Eigen3 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})


FILE(GLOB folder_source ./src/motorFrictionExcitationMain.cpp
                        ./src/motorFrictionExcitationModule.cpp
                        ./src/motorFrictionExcitationThread.cpp 
                        ./src/motorFrictionExcitationParams.cpp )
FILE(GLOB folder_header ./include/motorFrictionExcitation/motorFrictionExcitationModule.h
                        ./include/motorFrictionExcitation/motorFrictionExcitationThread.h
                        ./include/motorFrictionExcitation/motorFrictionExcitationConstants.h
                        ./include/motorFrictionExcitation/motorFrictionExcitationParams.h )

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

include(iCubHelpers)
include(iCubOptions)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${wholeBodyInterfaceIcub_INCLUDE_DIRS}
                    ${iDynTree_INCLUDE_DIRS}
                    ${Orocos-KDL_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS}
                    ${paramHelp_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} skinDynLib ctrlLib wholeBodyInterface wholeBodyInterfaceIcub paramHelp iDynTree ${Orocos-KDL_LIBRARIES} ${kdl_codyco_LIBRARIES})

if(WIN32)
	INSTALL_TARGETS(/bin/Release ${PROJECTNAME})	
else(WIN32)
	INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)

