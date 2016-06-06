# Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Marco Randazzo - marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME walkPlayer)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

set(HEADERS_DIR include/walkPlayer)

FILE(GLOB folder_source src/*.cpp)
# FILE(GLOB folder_header include/walkPlayer/scriptModule.h
#                         include/walkPlayer/workingThread.h
#                         include/walkPlayer/robotDriver.h
#                         include/walkPlayer/actionClass.h)
FILE(GLOB folder_header include/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

include_directories(SYSTEM ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${HEADERS_DIR})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} iKin iDyn)

if(WIN32)
    INSTALL_TARGETS(/bin/Release ${PROJECTNAME})
else(WIN32)
    INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)

add_subdirectory(app)
