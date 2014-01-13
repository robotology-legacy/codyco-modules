# Copyright (C) 2014 CoDyCo
# Author: Daniele Pucci, Francesco Romano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME adaptiveControl)
PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
find_package(Eigen3 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})


set(SRC_FOLDER ${PROJECT_SOURCE_DIR}/src)
set(INCLUDE_FOLDER ${PROJECT_SOURCE_DIR}/include)

set(SOURCES 
    ${SRC_FOLDER}/main.cpp 
    ${SRC_FOLDER}/AdaptiveControlModule.cpp 
    ${SRC_FOLDER}/AdaptiveControlThread.cpp)

set(INCLUDES 
    ${INCLUDE_FOLDER}/AdaptiveControlModule.h 
    ${INCLUDE_FOLDER}/AdaptiveControlConstants.h
    ${INCLUDE_FOLDER}/AdaptiveControlThread.h)

SOURCE_GROUP("Source Files" FILES ${SOURCES})
SOURCE_GROUP("Header Files" FILES ${INCLUDES})

include(iCubHelpers)
include(iCubOptions)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

INCLUDE_DIRECTORIES(${INCLUDE_FOLDER}
                    # ${motorFrictionIdentificationLib_INCLUDE_DIRS}
                    # ${wholeBodyInterface_INCLUDE_DIRS}
                    # ${wholeBodyInterfaceIcub_INCLUDE_DIRS}
                    ${iDynTree_INCLUDE_DIRS}
                    ${Orocos-KDL_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${paramHelp_INCLUDE_DIRS}
                    # ${skinDynLib_INCLUDE_DIRS}
                    # ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS} )

ADD_EXECUTABLE(${PROJECTNAME} ${SOURCES} ${INCLUDES})

TARGET_LINK_LIBRARIES(${PROJECTNAME} 
 # motorFrictionIdentificationLib
 ${YARP_LIBRARIES}
 # skinDynLib
#  ctrlLib
#  wholeBodyInterface
#  wholeBodyInterfaceIcub
 paramHelp
 iDynTree
 ${Orocos-KDL_LIBRARIES}
 ${kdl_codyco_LIBRARIES})

if(WIN32)
	INSTALL_TARGETS(/bin/Release ${PROJECTNAME})	
else(WIN32)
	INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)

# add_subdirectory(app)
