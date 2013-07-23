# Copyright: (C) 2013 CoDyCo Consortium
# Authors: Serena Ivaldi
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

SET(PROJECTNAME wbiy)

PROJECT(${PROJECTNAME})

find_package(ICUB)

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     IF(CMAKE_COMPILER_IS_GNUCC)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
     ENDIF(CMAKE_COMPILER_IS_GNUCC)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

SET(folder_source src/icubWholeBodyInterface.cpp 
                  src/icubWholeBodyModel.cpp
                  src/icubWholeBodyStates.cpp
                  src/yarpWholeBodyActuators.cpp
                  src/yarpWholeBodySensors.cpp)
SET(folder_header include/wbiy/wbiy.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
					  ${YARP_INCLUDE_DIRS}
                      ${skinDynLib_INCLUDE_DIRS}
                      ${wbi_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME}
                                     ${YARP_LIBRARIES}
                                     skinDynLib
                                     wbi)

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS wbi
                                   DESTINATION include/wbiy
                                   FILES ${folder_header})


