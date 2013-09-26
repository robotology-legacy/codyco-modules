# Copyright (C) 2012 RobotCub Consortium
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# YARP needs CMake 2.6 or greater
cmake_minimum_required(VERSION 2.6)

#add Wall
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

# find YARP and ICUB
find_package(YARP REQUIRED)
find_package(ICUB)
set( CMAKE_MODULE_PATH "/usr/local/share/yarp/cmake" ${CMAKE_MODULE_PATH})
find_package(GSL)

# add YARP and ICUB include directories
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

#load some macros useful for applications
include(iCubHelpers)

# default prefix for applications
SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(app)

