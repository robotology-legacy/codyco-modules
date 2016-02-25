# Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia
# Author: Jorhabib Eljaik, Yue Hu
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8)
project(iCubWalkingIK)

find_package(YARP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})

set(SRC_FOLDER src)
set(HEADERS_FOLDER include/iCubWalkingIK)

set(HEADERS ${HEADERS_FOLDER}/iCubWalkingIKModule.h
            ${HEADERS_FOLDER}/iCubWalkingIKThread.h)

SET(SOURCES ${SRC_FOLDER}/main.cpp
            ${SRC_FOLDER}/iCubWalkingIKModule.cpp
            ${SRC_FOLDER}/iCubWalkingIKThread.cpp)

source_group("Source Files" FILES ${SOURCES})
source_group("Header Files" FILES ${HEADERS})

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR}
                           ${wholeBodyInterface_INCLUDE_DIRS}
                           ${yarpWholeBodyInterface_INCLUDE_DIRS}
                           ${YARP_INCLUDE_DIRS})

include_directories(${HEADERS_FOLDER})

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME}
                      ${wholeBodyInterface_LIBRARIES}
                      ${yarpWholeBodyInterface_LIBRARIES}
                      ${YARP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_subdirectory(app)
