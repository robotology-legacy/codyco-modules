# Copyright: CoDyCo Consortium (2013-..)
# Author: Andrea Del Prete - andrea.delprete@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME paramHelp)

PROJECT(${PROJECTNAME})

SET(folder_source src/paramHelp.cpp
                  src/paramHelpClient.cpp
                  src/paramHelpServer.cpp)

SET(folder_header include/paramHelp/paramHelp.h
                  include/paramHelp/paramHelpClient.h
                  include/paramHelp/paramHelpServer.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS} )

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS 
                                   DESTINATION include/paramHelp
                                   FILES ${folder_header})
