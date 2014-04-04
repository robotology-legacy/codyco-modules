# Copyright (C) 2013 CoDyCo
# Author: Jorhabib Eljaik
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# MATLAB and YARP is searched before adding the different modules because they will be used by all of them
find_package(YARP       REQUIRED)
find_package(Matlab	 REQUIRED)

if(MATLAB_FOUND)
        message(STATUS "Matlab was found!! The configuration may continue")
        set(MATLAB_LIBRARIES
                ${MATLAB_MEX_LIBRARY}
                ${MATLAB_MX_LIBRARY}
                ${MATLAB_ENG_LIBRARY}
        )
else()
        message(FATAL_ERROR "Matlab not found")
endif()


#add_subdirectory(wbInterface)
add_subdirectory(minJerkGenerator)
add_subdirectory(yRead)
add_subdirectory(yWrite)
add_subdirectory(softRealTime)
add_subdirectory(wbInterfaceNEW)
add_subdirectory(ySynchronizer)
