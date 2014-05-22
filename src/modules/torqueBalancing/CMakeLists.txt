# Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia
# Author: Francesco Romano,
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8)
project(torqueBalancing)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(paramHelp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(wholeBodyInterface REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

if(${CODYCO_USES_URDFDOM})
    find_package(Boost)
endif()
# find_package(ICUB)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

set(SRC_FOLDER src)
set(HEADERS_FOLDER include/torquebalancing)

set(HEADERS    ${HEADERS_FOLDER}/TorqueBalancingModule.h
               ${HEADERS_FOLDER}/TorqueBalancingController.h
               ${HEADERS_FOLDER}/ReferenceGenerator.h
               ${HEADERS_FOLDER}/ReferenceGeneratorInputReaderImpl.h
               ${HEADERS_FOLDER}/Reference.h
               ${HEADERS_FOLDER}/SmootherInputReader.h
               ${HEADERS_FOLDER}/config.h
               ${HEADERS_FOLDER}/ParamHelperConfig.h)

set(SOURCES    ${SRC_FOLDER}/TorqueBalancingModule.cpp
               ${SRC_FOLDER}/TorqueBalancingController.cpp
               ${SRC_FOLDER}/ReferenceGenerator.cpp
               ${SRC_FOLDER}/ReferenceGeneratorInputReaderImpl.cpp
               ${SRC_FOLDER}/SmootherInputReader.cpp
               ${SRC_FOLDER}/config.cpp
               ${SRC_FOLDER}/Reference.cpp
               ${SRC_FOLDER}/main.cpp)

source_group("Source Files" FILES ${SOURCES})
source_group("Header Files" FILES ${HEADERS})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${yarpWholeBodyInterface_INCLUDE_DIRS}
                    ${paramHelp_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${codyco-commons_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${SOURCES} ${HEADERS})

TARGET_LINK_LIBRARIES(${PROJECTNAME}
                      ${wholeBodyInterface_LIBRARIES}
                      ${yarpWholeBodyInterface_LIBRARIES}
                      ${paramHelp_LIBRARIES}
                      ${ctrlLib_LIBRARIES}
                      ${YARP_LIBRARIES}
                      ${codyco-commons_LIBRARIES})

if(WIN32)
    INSTALL_TARGETS(/bin/Release ${PROJECTNAME})
else(WIN32)
    INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)


# install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# add_subdirectory(app)
