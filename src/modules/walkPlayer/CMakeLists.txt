# Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Marco Randazzo - marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME walkPlayer)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
set(HEADERS_DIR include/walkPlayer)

FILE(GLOB folder_source src/*.cpp)
# FILE(GLOB folder_header include/walkPlayer/scriptModule.h
#                         include/walkPlayer/workingThread.h
#                         include/walkPlayer/robotDriver.h
#                         include/walkPlayer/actionClass.h)
FILE(GLOB folder_header include/*.h)

message("HEADERS THAT WILL BE ADDED FOR WALKPLAYER: " ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

include(iCubHelpers)
include(iCubOptions)


INCLUDE_DIRECTORIES(${iDyn_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${HEADERS_DIR})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iDyn
                                     ${YARP_LIBRARIES})

if(WIN32)
    INSTALL_TARGETS(/bin/Release ${PROJECTNAME})
else(WIN32)
    INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)

add_subdirectory(app)
