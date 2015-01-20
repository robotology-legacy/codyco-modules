# Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Jorhabib Eljaik jorhabib.eljaik@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME wholeBodyNeckVelocity)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

set(HEADERS_DIR include/wholeBodyNeckVelocity)

FILE(GLOB SRC_FILES src/*.cpp)
FILE(GLOB HEADER_FILES include/wholeBodyNeckVelocity/wholeBodyNeckVelocity.h)

source_group("Source Files" FILES ${SRC_FILES})
source_group("Header Files" FILES ${HEADER_FILES})

include_directories(${HEADERS_DIR})

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR}
                           ${wholeBodyInterface_INCLUDE_DIRS}
                           ${yarpWholeBodyInterface_INCLUDE_DIRS}
                           ${YARP_INCLUDE_DIRS})
                           
add_executable(${PROJECTNAME} ${SRC_FILES} ${HEADER_FILES})

target_link_libraries(${PROJECTNAME}
                      ${wholeBodyInterface_LIBRARIES}
                      ${yarpWholeBodyInterface_LIBRARIES}
                      ${YARP_LIBRARIES})
                      
if(WIN32)
    INSTALL_TARGETS(/bin/Release ${PROJECTNAME})	
else(WIN32)
    INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)

