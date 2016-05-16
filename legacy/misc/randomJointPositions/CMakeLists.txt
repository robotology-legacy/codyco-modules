# Copyright: 2012 iCub Facility, Istituto Italiano di Tecnologia
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME randomJointPositions)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

include_directories(${YARP_INCLUDE_DIRS})
add_executable(randomJointPositions randomJointPositions.cpp)
target_link_libraries(randomJointPositions ${YARP_LIBRARIES})


