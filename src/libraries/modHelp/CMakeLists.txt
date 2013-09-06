# Copyright: MACSi Consortium (2011-2013) & CoDyCo Consortium (2013)
# Author: Serena Ivaldi - serena.ivaldi@isir.upmc.fr
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME modHelp)

PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(Eigen3)

SET(folder_source src/modHelp.cpp)
SET(folder_header include/modHelp/modHelp.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

SET(EIGEN_UNSUPPORTED_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/extern/eigen_unsupported 
				${CMAKE_SOURCE_DIR}/extern/eigen_unsupported/Eigen/src)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${iCub_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${EIGEN_UNSUPPORTED_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib)

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS ctrlLib
                                   DESTINATION include/modHelp
                                   FILES ${folder_header})
