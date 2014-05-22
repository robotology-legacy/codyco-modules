# Copyright (C) 2013 CoDyCo
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME motorFrictionIdentification)
PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
find_package(Eigen3 REQUIRED)

FILE(GLOB folder_source ./src/motorFrictionIdentificationMain.cpp
                        ./src/motorFrictionIdentificationModule.cpp
                        ./src/motorFrictionIdentificationThread.cpp
                        ./src/recursiveLinearEstimator.cpp
                        ./src/alternatingRecursiveLinearEstimator.cpp )
FILE(GLOB folder_header ./include/motorFrictionIdentification/motorFrictionIdentificationModule.h
                        ./include/motorFrictionIdentification/motorFrictionIdentificationThread.h
                        ./include/motorFrictionIdentification/motorFrictionIdentificationConstants.h
                        ./include/motorFrictionIdentification/recursiveLinearEstimator.h
                        ./include/motorFrictionIdentification/alternatingRecursiveLinearEstimator.h )

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${paramHelp_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME}
                      ${wholeBodyInterface_LIBRARIES}
                      ${yarpWholeBodyInterface_LIBRARIES}
                      ${paramHelp_LIBRARIES}
                      ${ctrlLib_LIBRARIES}
                      ${YARP_LIBRARIES})

if(WIN32)
    INSTALL_TARGETS(/bin/Release ${PROJECTNAME})
else(WIN32)
    INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)

