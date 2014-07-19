# Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME staticInertiaIdentification)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(paramHelp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
find_package(TinyXML REQUIRED)

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${iDynTree_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${TinyXML_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

#Thrift configuration
include(YarpIDL)

set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}")
yarp_idl_to_dir(staticInertiaIdentification.thrift ${generated_libs_dir})


ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES}
                                     ${TinyXML_LIBRARIES}
                                     ${iDynTree_LIBRARIES}
                                     ${kdl_codyco_LIBRARIES)
                                     ${wholeBodyInterface_LIBRARIES}
                                     ${yarpWholeBodyInterface_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

add_subdirectory(app)
