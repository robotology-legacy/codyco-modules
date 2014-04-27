# Copyright (C) 2013 CoDyCo Consortium
# Author: Serena Ivaldi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 2.8)

SET(PROJECTNAME ISIRControllerYarpTest2)
PROJECT(${PROJECTNAME} CXX)


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-fpermissive")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")


# find packages
#--------------------------

find_package(YARP)
find_package(ICUB)

# we also have unsupported files of eigen
find_package(Eigen3)

# now stuff related to ISIR_controller
find_package(PkgConfig REQUIRED)

PKG_CHECK_MODULES(PC_ORCISIR_ISIRCONTROLLER orcisir_ISIRController REQUIRED)
if(NOT PC_ORCISIR_ISIRCONTROLLER_FOUND)
    message("Failed to detect orcisir_ISIRController, check PKG_CONFIG_PATH." )
endif(NOT PC_ORCISIR_ISIRCONTROLLER_FOUND)

# debug messages to find correctly ISIRController
message("PC_ORCISIR_ISIRCONTROLLER_INCLUDE_DIRS =  ${PC_ORCISIR_ISIRCONTROLLER_INCLUDE_DIRS}")
message("PC_ORCISIR_ISIRCONTROLLER_LIBRARIES =  ${PC_ORCISIR_ISIRCONTROLLER_LIBRARIES}")
message("PC_ORCISIR_ISIRCONTROLLER_LIBRARY_DIRS =  ${PC_ORCISIR_ISIRCONTROLLER_LIBRARY_DIRS}")


# source directories
#--------------------------

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./src/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


# include directories
#--------------------------

include(iCubHelpers)
include(iCubOptions)

# these are necessary and taken from $CODYCO_ROOT/extern
SET(EIGEN_UNSUPPORTED_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/extern/eigen_unsupported 
				                       ${CMAKE_SOURCE_DIR}/extern/eigen_unsupported/Eigen/src)
				
INCLUDE_DIRECTORIES(${iDyn_INCLUDE_DIRS}
                    ${modHelp_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${BOOST_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${EIGEN_UNSUPPORTED_INCLUDE_DIRS}
                    ${PC_ORCISIR_ISIRCONTROLLER_INCLUDE_DIRS}
                    #${PC_EIGEN_INCLUDE_DIRS}                    
                    )
                    				
# link libraries
#--------------------------

LINK_DIRECTORIES(${PC_ORCISIR_ISIRCONTROLLER_LIBRARY_DIRS})

#this must go always before link_directories
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iDyn
                                     icubmod
                                     modHelp
                                     ${YARP_LIBRARIES}
                                     ${PC_ORCISIR_ISIRCONTROLLER_LIBRARIES}
                                     )
                                     
# install
#---------------------------------------

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT) 
SET(CMAKE_INSTALL_PREFIX $ENV{ICUB_DIR} CACHE PATH "installation prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

if(WIN32)
	INSTALL_TARGET(/bin/Release ${PROJECTNAME})	
else(WIN32)
	INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)

#add_subdirectory(app)



