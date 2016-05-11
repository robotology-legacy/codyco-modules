# Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia
# Author: Francesco Romano,
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8)
project(wholeBodySensorTester)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(paramHelp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface 0.2.2 REQUIRED)
find_package(codycoCommons 0.1 REQUIRED)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
# 
# set(SRC_FOLDER src)
set(HEADERS_FOLDER include)

FILE(GLOB folder_source src/*.cpp)
# FILE(GLOB folder_header include/walkPlayer/scriptModule.h
#                         include/walkPlayer/workingThread.h
#                         include/walkPlayer/robotDriver.h
#                         include/walkPlayer/actionClass.h)
FILE(GLOB folder_header include/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# set(HEADERS    ${HEADERS_FOLDER}/TorqueBalancingModule.h
#                ${HEADERS_FOLDER}/TorqueBalancingController.h
#                ${HEADERS_FOLDER}/ReferenceGenerator.h
#                ${HEADERS_FOLDER}/ReferenceGeneratorInputReaderImpl.h
#                ${HEADERS_FOLDER}/Reference.h
#                ${HEADERS_FOLDER}/MinimumJerkTrajectoryGenerator.h
#                ${HEADERS_FOLDER}/config.h
#                ${HEADERS_FOLDER}/ParamHelperConfig.h
#                ${HEADERS_FOLDER}/DynamicConstraint.h)
# 
# set(SOURCES    ${SRC_FOLDER}/TorqueBalancingModule.cpp
#                ${SRC_FOLDER}/TorqueBalancingController.cpp
#                ${SRC_FOLDER}/ReferenceGenerator.cpp
#                ${SRC_FOLDER}/ReferenceGeneratorInputReaderImpl.cpp
#                ${SRC_FOLDER}/MinimumJerkTrajectoryGenerator.cpp
#                ${SRC_FOLDER}/config.cpp
#                ${SRC_FOLDER}/Reference.cpp
#                ${SRC_FOLDER}/main.cpp
#                ${SRC_FOLDER}/DynamicConstraint.cpp)
# 
# source_group("Source Files" FILES ${SOURCES})
# source_group("Header Files" FILES ${HEADERS})

add_definitions(-D_USE_MATH_DEFINES)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR}
    ${wholeBodyInterface_INCLUDE_DIRS}
    ${yarpWholeBodyInterface_INCLUDE_DIRS}
    ${paramHelp_INCLUDE_DIRS}
    ${skinDynLib_INCLUDE_DIRS}
    ${ctrlLib_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    ${codycoCommons_INCLUDE_DIRS})

include_directories(${HEADERS_FOLDER} ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECT_NAME}
                      ${wholeBodyInterface_LIBRARIES}
                      ${yarpWholeBodyInterface_LIBRARIES}
                      ${paramHelp_LIBRARIES}
                      ${codycoCommons_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_subdirectory(app)
# 
# if(CODYCO_BUILD_TESTS)
#     add_subdirectory(tests)
# endif()
