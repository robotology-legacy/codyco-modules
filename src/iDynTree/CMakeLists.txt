# Copyright: (C) 2013 RobotCub Consortium
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

SET(PROJECTNAME iDynTree)

PROJECT(${PROJECTNAME})

SET(folder_source )

SET(folder_header include/iCub/iDynTree/iDynTree.h
                  include/iCub/iDynTree/iDynTreeInterface.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${skinDynLib_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iKin
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS skinDynLib
                                   DESTINATION include/iCub/iDynTree
                                   FILES ${folder_header})


