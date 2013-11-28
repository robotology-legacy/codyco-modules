# Copyright (C) 2013 CoDyCo
# Author: Jorhabib Eljaik
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
SET(PROJECTNAME yWrite)
PROJECT(${PROJECTNAME})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(YARP)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

#User Directories
set(MATLAB_PATH "/usr/local/MATLAB/R2012a" CACHE PATH "Matlab Installation Path")

include_directories("${MATLAB_PATH}/simulink/include")
include_directories("${MATLAB_PATH}/extern/include")
include_directories("${YARP_INCLUDE_DIRS}")
include_directories(${PROJECT_SOURCE_DIR}/include)

add_definitions(-DMATLAB_MEX_FILE)

if(APPLE)
    set(SFUNCTION_SUFFIX ".mexmaci64")
elseif(UNIX)
    set(SFUNCTION_SUFFIX ".mexa86")
elseif(WIN32)
    set(SFUNCTION_SUFFIX ".mexw64")
endif()

link_directories(${CMAKE_BINARY_DIR})
link_directories("${MATLAB_PATH}/bin/glnxa86")

add_library(${PROJECTNAME} SHARED ${folder_source} ${header_source})

set(CMAKE_CXX_FLAGS "-fpermissive")

#Environmental variable MATLAB_ROOT pointing to installation directory of MATLAB should be set
if(NOT $ENV{MATLAB_ROOT} STREQUAL "")
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
                        set_target_properties(${PROJECTNAME} PROPERTIES SUFFIX .mexa64)
                        set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_FLAGS "-I$ENV{MATLAB_ROOT} -O -pthread -shared -Wl,--version-script,$ENV{MATLAB_ROOT}/extern/lib/glnxa64/mexFunction.map -Wl,--no-undefined -Wl,-rpath-link,$ENV{MATLAB_ROOT}/bin/glnxa64 -L$ENV{MATLAB_ROOT}/bin/glnxa64 -lmx -lmex -lmat -lm -lstdc++")
	else()
                        set_target_properties(${PROJECTNAME} PROPERTIES SUFFIX .mexglx
										   PREFIX "")
                        set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_FLAGS "-I$ENV{MATLAB_ROOT} -O -pthread -shared -m32 -Wl,--version-script,$ENV{MATLAB_ROOT}/extern/lib/glnx86/mexFunction.map -Wl,--no-undefined -Wl,-rpath-link,$ENV{MATLAB_ROOT}/bin/glnx86 -L$ENV{MATLAB_ROOT}/bin/glnx86 -lmx -lmex -lmat -lm -lstdc++")
	endif()		

        target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
        install(TARGETS ${PROJECTNAME} DESTINATION mex)
endif()



