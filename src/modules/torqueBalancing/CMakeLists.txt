# Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia
# Author: Francesco Romano, 
# CopyPolicy: Released under the terms of the GNU GPL v2.0 or any later version.

cmake_minimum_required(VERSION 2.8)
project(torqueBalancing)

find_package(YARP)
find_package(Eigen3 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

set(SRC_FOLDER src)
set(HEADERS_FOLDER include/torquebalancing)

set(HEADERS    ${HEADERS_FOLDER}/TorqueBalancingModule.h
               ${HEADERS_FOLDER}/TorqueBalancingController.h
               ${HEADERS_FOLDER}/ReferenceGenerator.h
               ${HEADERS_FOLDER}/ReferenceGeneratorInputReaderImpl.h
               ${HEADERS_FOLDER}/Reference.h
               ${HEADERS_FOLDER}/config.h
               ${HEADERS_FOLDER}/ParamHelperConfig.h)

set(SOURCES    ${SRC_FOLDER}/TorqueBalancingModule.cpp
               ${SRC_FOLDER}/TorqueBalancingController.cpp
               ${SRC_FOLDER}/ReferenceGenerator.cpp
               ${SRC_FOLDER}/ReferenceGeneratorInputReaderImpl.cpp
               ${SRC_FOLDER}/config.cpp
               ${SRC_FOLDER}/Reference.cpp
               ${SRC_FOLDER}/main.cpp)
                               
source_group("Source Files" FILES ${SOURCES})
source_group("Header Files" FILES ${HEADERS})

message("Codyco common: ${codyco_commons_INCLUDE_DIRS}")

include_directories(${HEADERS_FOLDER}
                    ${codyco_commons_INCLUDE_DIRS}
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${wholeBodyInterfaceIcub_INCLUDE_DIRS}
                    ${paramHelp_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(  ${PROJECT_NAME} 
                        codyco_commons
                        wholeBodyInterface
                        wholeBodyInterfaceIcub
                        paramHelp
                        ${YARP_LIBRARIES})

# install(TARGETS ${PROJECTNAME} DESTINATION bin)

# add_subdirectory(app)
