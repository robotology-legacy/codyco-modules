# Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME wholeBodyDynamicsTree)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(iDynTree REQUIRED)

#Thrift configuration
include(YarpIDL)

set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}")
yarp_idl_to_dir(wholeBodyDynamics.thrift ${generated_libs_dir})

find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/wholeBodyDynamicsTree/*.h ./include/ctrlLibRT/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(SYSTEM  ${wholeBodyInterface_INCLUDE_DIRS}
                            ${yarpWholeBodyInterface_INCLUDE_DIRS}
                            ${iDynTree_INCLUDE_DIRS}
                            ${skinDynLib_INCLUDE_DIRS}
                            ${ctrlLib_INCLUDE_DIRS}
                            ${YARP_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIR})


ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

message(STATUS "iDynTree_LIBRARIES: " ${iDynTree_LIBRARIES})
message(STATUS "yarpWholeBodyInterface_LIBRARIES: " ${yarpWholeBodyInterface_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES}
                                     ${iDynTree_LIBRARIES}
                                     ${wholeBodyInterface_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
