# Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8.11)
SET(PROJECTNAME reachRandomJointPositions)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iDynTree REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./src/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

include_directories(SYSTEM ${YARP_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/src)
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
