# Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME wholeBodyDynamicsTree)
PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/wholeBodyDynamicsTree/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${wholeBodyInterfaceIcub_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${iDynTree_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

#Thrift configuration
include(YarpIDL)

set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}")
yarp_idl_to_dir(wholeBodyDynamics.thrift ${generated_libs_dir})


ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} wholeBodyInterface wholeBodyInterfaceIcub iDynTree)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

