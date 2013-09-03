# Copyright (C) 2013 CoDyCo Consortium
# Author: Serena Ivaldi
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 2.8)
SET(PROJECTNAME ISIRControllerYarpTest)
PROJECT(${PROJECTNAME} CXX)


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-fpermissive")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})


# find packages
#--------------------------

find_package(YARP)
find_package(ICUB)


# we have unsupported files of eigen
find_package(Eigen3)
#PKG_CHECK_MODULES(PC_EIGEN eigen3 REQUIRED)
#if(NOT PC_EIGEN_FOUND)
#    message("Failed to detect Eigen, check PKG_CONFIG_PATH." )
#endif(NOT PC_EIGEN_FOUND)

# now stuff related to ISIR_controller
find_package(PkgConfig REQUIRED)
PKG_CHECK_MODULES(PC_ORCISIR_ISIRCONTROLLER orcisir_ISIRController REQUIRED)
if(NOT PC_ORCISIR_ISIRCONTROLLER_FOUND)
    message("Failed to detect orcisir_ISIRController, check PKG_CONFIG_PATH." )
endif(NOT PC_ORCISIR_ISIRCONTROLLER_FOUND)


# include directories
#--------------------------

include_directories(
    ${PC_ORCISIR_ISIRCONTROLLER_INCLUDE_DIRS}
    ${PC_EIGEN_INCLUDE_DIRS}
)


# link libraries
#--------------------------

LINK_DIRECTORIES(
    ${PC_ORCISIR_ISIRCONTROLLER_LIBRARY_DIRS}
)

#========================================
add_library(
    Model3T
    SHARED
    Model3T.cpp
)

target_link_libraries(
    Model3T
    ${PC_ORCISIR_ISIRCONTROLLER_LIBRARIES}
)

#========================================
add_executable(
    test_3T
    main.cpp
)

target_link_libraries(
    test_3T
    Model3T
)

