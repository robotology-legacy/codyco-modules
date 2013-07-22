# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME wholeBodyInterfaceYarpTest)
PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

FILE(GLOB folder_source ./*.cpp)
FILE(GLOB folder_header ./*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

include(iCubHelpers)
include(iCubOptions)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS}
                    ${wbi_INCLUDE_DIRS}
                    ${wbiy_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} skinDynLib wbi wbiy)

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT) 
SET(CMAKE_INSTALL_PREFIX $ENV{ICUB_DIR} CACHE PATH "installation prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
if(WIN32)
	INSTALL_TARGET(/bin/Release ${PROJECTNAME})	
else(WIN32)
	INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)
