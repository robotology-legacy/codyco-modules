# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME wholeBodyInterfaceTest)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(wholeBodyInterface REQUIRED)


FILE(GLOB folder_source ./src/main.cpp
                        ./src/testRotation.cpp
                        ./src/testFrame.cpp
                        ./src/testUtil.cpp )
FILE(GLOB folder_header ./include/wbiTest/testRotation.h
                        ./include/wbiTest/testFrame.h
                        ./include/wbiTest/testUtil.h )

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})


TARGET_LINK_LIBRARIES(${PROJECTNAME} ${wholeBodyInterface_LIBRARIES}
                                     ${YARP_LIBRARIES})

if(WIN32)
    INSTALL_TARGETS(/bin/Release ${PROJECTNAME})
else(WIN32)
    INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)
