# Copyright (C) 2015 CoDyCo
# Author: Jorhabib Eljaik
# CopyPolicy: Released under the terms of the GNU GPL v2.0

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME wholeBodyEstimator)
project(${PROJECTNAME})

find_package(YARP   REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(MATRIX REQUIRED)
# The following PkgConfig is for searching OROCOS_BFL
# Note: The way OROCOS_BFL is searched and so on might be necessary to get updated with more recent CMake directives.
find_package(PkgConfig)
pkg_check_modules(OROCOS_BFL REQUIRED orocos-bfl)

file(GLOB source_dir    src/main.cpp
                        src/WholeBodyEstimatorModule.cpp
                        src/WholeBodyEstimatorThread.cpp
                        src/QuaternionEKF.cpp
                        src/SingleLeggedOdometry.cpp)
file(GLOB header_dir    include/WholeBodyEstimatorModule.h
	                    include/WholeBodyEstimatorThread.h
                        include/IEstimator.h
                        include/QuaternionEKF.h
                        include/SingleLeggedOdometry.h)

source_group("Source Files" FILES ${source_dir})
source_group("Header Files" FILES ${header_dir})

include_directories(SYSTEM ${YARP_INCLUDE_DIRS}
                           ${MATRIX_INCLUDE}
                           ${OROCOS_BFL_INCLUDEDIR}
					       ${PROJECT_SOURCE_DIR}/include)

message("** Matrix include dirs are: " ${MATRIX_INCLUDE})
message("** Orocos-bfl include dirs are: " ${OROCOS_BFL_INCLUDEDIR})

link_directories(${OROCOS_BFL_LIBRARY_DIRS})

message("${PROJECTNAME} ${source_dir} ${header_dir}")
add_executable(${PROJECTNAME} ${source_dir} ${header_dir})


target_link_libraries(${PROJECTNAME}
                      ${OROCOS_BFL_LIBRARIES}
	                  ${YARP_LIBRARIES}
				      ${MATRIX_LIBS})

if(WIN32)
INSTALL_TARGETS(/bin/Release ${PROJECTNAME})
else(WIN32)
    INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)

add_subdirectory(app)
