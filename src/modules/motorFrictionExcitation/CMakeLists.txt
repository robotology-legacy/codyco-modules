# Copyright (C) 2013 CoDyCo
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME motorFrictionExcitation)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(paramHelp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
find_package(motorFrictionIdentificationLib REQUIRED)

FILE(GLOB folder_source ./src/motorFrictionExcitationMain.cpp
                        ./src/motorFrictionExcitationModule.cpp
                        ./src/motorFrictionExcitationThread.cpp  )
FILE(GLOB folder_header ./include/motorFrictionExcitation/motorFrictionExcitationModule.h
                        ./include/motorFrictionExcitation/motorFrictionExcitationThread.h
                        ./include/motorFrictionExcitation/motorFrictionExcitationConstants.h )

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${paramHelp_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${motorFrictionIdentificationLib_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME}
                      ${motorFrictionIdentificationLib_LIBRARIES}
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

