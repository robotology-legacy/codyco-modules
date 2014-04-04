# Copyright: CoDyCo Consortium (2013-..)
# Author: Andrea Del Prete - andrea.delprete@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME motorFrictionIdentificationLib)

PROJECT(${PROJECTNAME})

SET(folder_source src/motorFrictionExcitationParams.cpp )

SET(folder_header include/motorFrictionIdentificationLib/motorFrictionIdentificationParams.h
                  include/motorFrictionIdentificationLib/motorFrictionExcitationParams.h
                  include/motorFrictionIdentificationLib/jointTorqueControlParams.h )

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${paramHelp_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIR})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${paramHelp_LIBRARIES} ${YARP_LIBRARIES})

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS 
                                   DESTINATION include/motorFrictionIdentificationLib
                                   FILES ${folder_header})
