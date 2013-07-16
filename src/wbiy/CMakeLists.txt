# Copyright: (C) 2013 CoDyCo Consortium
# Authors: Serena Ivaldi
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

SET(PROJECTNAME wbiy)

PROJECT(${PROJECTNAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

SET(folder_source src/wbiy.cpp)
SET(folder_header include/wbiy/wbiy.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
					  ${YARP_INCLUDE_DIRS}
                                        ${wbi_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME}
                                     ${YARP_LIBRARIES}
                                     wbi)

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS wbi
                                   DESTINATION include/wbiy
                                   FILES ${folder_header})


