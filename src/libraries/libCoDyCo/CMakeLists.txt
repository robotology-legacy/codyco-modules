# Copyright: (C) 2013 RobotCub Consortium
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

cmake_minimum_required( VERSION 2.8 FATAL_ERROR )


SET(PROJECTNAME libCoDyCo)

PROJECT(${PROJECTNAME})

# set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

SET(SRCS  )

SET(HEADERS  include/Common.h)
                          
SOURCE_GROUP("Source Files" FILES ${SRCS})
SOURCE_GROUP("Header Files" FILES ${HEADERS})

INCLUDE_DIRECTORIES(include)
                    
ADD_LIBRARY(${PROJECTNAME} ${SRCS} ${HEADERS})

TARGET_LINK_LIBRARIES(${PROJECTNAME} )

# 
# icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
#                                    DESTINATION include/iCub/iDynTree
#                                    FILES ${folder_header})
                                  
