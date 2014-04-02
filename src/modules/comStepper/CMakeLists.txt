# Copyright (C) 2010 RobotCub Consortium
# Author: Jorhabib Eljaik, Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME comStepper)
PROJECT(${PROJECTNAME})

SET(folder_source ./src/main.cpp ./src/comStepper.cpp ./src/rangeCheck.cpp ./src/util.cpp)
SET(folder_header ./src/comStepper.h ./src/rangeCheck.h ./src/util.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

include(iCubHelpers)
include(iCubOptions)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

INCLUDE_DIRECTORIES(${iDyn_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${iDyn_LIBRARIES} ${YARP_LIBRARIES}
${ctrlLib_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
