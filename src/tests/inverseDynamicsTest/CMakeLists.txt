# Copyright (C) 2010 RobotCub Consortium
# Author: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME inverseDynamicsTest)
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


IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     IF(CMAKE_COMPILER_IS_GNUCC)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
     ENDIF(CMAKE_COMPILER_IS_GNUCC)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")


INCLUDE_DIRECTORIES(
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${wholeBodyInterfaceIcub_INCLUDE_DIRS}
                    ${iDynTree_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${Orocos-KDL_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${skinDynLib_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} wholeBodyInterface wholeBodyInterfaceIcub
${YARP_LIBRARIES} skinDynLib ${iDynTree_LIBRARIES})

if(WIN32)
	INSTALL_TARGETS(/bin/Release ${PROJECTNAME})
else(WIN32)
	INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)
