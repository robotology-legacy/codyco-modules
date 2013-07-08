# Copyright: (C) 2013 RobotCub Consortium
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

cmake_minimum_required( VERSION 2.8 FATAL_ERROR )


SET(PROJECTNAME iDynTree)

PROJECT(${PROJECTNAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


find_package(YARP)
find_package(ICUB)
find_package(Eigen3 REQUIRED)
find_package(Orocos-KDL REQUIRED)
find_package(kdl_codyco REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

SET(folder_source src/iDynTree.cpp
				  src/yarp_kdl.cpp)

SET(folder_header include/iCub/iDynTree/iDynTree.h
                  include/iCub/iDynTree/iDynTreeInterface.h
                  include/iCub/iDynTree/yarp_kdl.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})



INCLUDE_DIRECTORIES(include
                    ${skinDynLib_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME}
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES}
                                     ${Orocos-KDL_LIBRARIES} 
                                     ${kdl_codyco_LIBRARIES})

#icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
#                                   DEPENDS skinDynLib
#                                   DESTINATION include/iCub/iDynTree
#                                   FILES ${folder_header})


