# Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME wholeBodyDynamicsTree)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(iDynTree REQUIRED)

find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

file(GLOB folder_source ./src/*.cpp)
file(GLOB folder_header ./include/wholeBodyDynamicsTree/*.h ./include/ctrlLibRT/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# This is a deprecated module using deprecated parts of iDynTree, removing deprecating warnings
codyco_disable_deprecation_warnings()

add_definitions(-D_USE_MATH_DEFINES)

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(SYSTEM  ${wholeBodyInterface_INCLUDE_DIRS}
                            ${yarpWholeBodyInterface_INCLUDE_DIRS}
                            ${iDynTree_INCLUDE_DIRS}
                            ${skinDynLib_INCLUDE_DIRS}
                            ${ctrlLib_INCLUDE_DIRS}
                            ${YARP_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIR})


add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES}
                                     ${iDynTree_LIBRARIES}
                                     ${wholeBodyInterface_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES}
                                     wholeBodyDynamics_IDLServer
                                     ctrlLibRT)

install(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
