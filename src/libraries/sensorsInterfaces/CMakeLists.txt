# Copyright: (C) 2013 CoDyCo Consortium
# Authors: Serena Ivaldi
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

SET(PROJECTNAME sensorsInterfaces)

PROJECT(${PROJECTNAME})

SET(folder_source src/sensorsInterfaces.cpp)
SET(folder_header include/sensorsInterfaces/sensorsInterfaces.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME})

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DESTINATION include/sensorsInterfaces
                                   FILES ${folder_header})


