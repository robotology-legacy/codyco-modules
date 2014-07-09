# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
#PROJECT(inverseDynamicsTestProject)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

FILE(GLOB folder_source ./*.cpp)
FILE(GLOB folder_header ./*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


INCLUDE_DIRECTORIES(${yarpWholeBodyInterface_INCLUDE_DIRS})

ADD_EXECUTABLE(inverseDynamicsTest ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(inverseDynamicsTest ${yarpWholeBodyInterface_LIBRARIES})

if(WIN32)
	INSTALL_TARGETS(/bin/Release inverseDynamicsTest)
else(WIN32)
	INSTALL_TARGETS(/bin inverseDynamicsTest)
endif(WIN32)
