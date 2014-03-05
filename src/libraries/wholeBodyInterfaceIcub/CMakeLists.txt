# Copyright: (C) 2013 CoDyCo Consortium
# Authors: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

SET(PROJECTNAME wholeBodyInterfaceIcub)

PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
find_package(kdl_codyco REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     IF(CMAKE_COMPILER_IS_GNUCC)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
     ENDIF(CMAKE_COMPILER_IS_GNUCC)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

SET(folder_source src/wbiIcubUtil.cpp
                  src/icubWholeBodyInterface.cpp 
                  src/icubWholeBodyModel.cpp
                  src/icubWholeBodyStates.cpp
                  src/icubWholeBodyStatesLocal.cpp
                  src/icubWholeBodyActuators.cpp
                  src/icubWholeBodySensors.cpp)
SET(folder_header include/wbiIcub/wholeBodyInterfaceIcub.h
                  include/wbiIcub/icubWholeBodyModel.h
                  include/wbiIcub/icubWholeBodyStates.h
                  include/wbiIcub/icubWholeBodyStatesLocal.h
                  include/wbiIcub/icubWholeBodyStatesRemote.h
                  include/wbiIcub/icubWholeBodyActuators.h
                  include/wbiIcub/icubWholeBodySensors.h 
                  include/wbiIcub/wbiIcubUtil.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

include(iCubHelpers)
include(iCubOptions)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${wholeBodyInterface_INCLUDE_DIRS}
                    ${iDynTree_INCLUDE_DIRS}
                    ${motorFrictionIdentificationLib_INCLUDE_DIRS}  #this is a temporary requirement
                    ${paramHelp_INCLUDE_DIRS} #this is a temporary requirement
                    ${kdl_codyco_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}) 

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} 
                        wholeBodyInterface
                        iDynTree
                        paramHelp #this is a temporary requirement
                        ${YARP_LIBRARIES}
                        skinDynLib
                        ctrlLib
                        ${kdl_codyco_LIBRARIES}
                        ${ctrlLib_LIBRARIES})

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DEPENDS wholeBodyInterface
                                   DESTINATION include/wbiIcub
                                   FILES ${folder_header})


