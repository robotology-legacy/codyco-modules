# Copyright (C) 2015 CoDyCo
# Author: Jorhabib Eljaik
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME quaternionEKF)
project(${PROJECTNAME})

find_package(YARP   REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(MATRIX REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(ICUB REQUIRED)
find_package(Eigen3)

# The following PkgConfig is for searching OROCOS_BFL
find_package(PkgConfig)
pkg_check_modules(OROCOS_BFL REQUIRED orocos-bfl)

file(GLOB source_dir src/dataDumperParser.cpp
                        src/directFilterComputation.cpp
                        src/main.cpp
                        src/nonLinearAnalyticConditionalGaussian.cpp
                        src/nonLinearMeasurementGaussianPdf.cpp
                        src/quaternionEKFModule.cpp
                        src/quaternionEKFThread.cpp)
file(GLOB header_dir include/dataDumperParser.h
                        include/directFilterComputation.h
                        include/nonLinearAnalyticConditionalGaussian.h
                        include/nonLinearMeasurementGaussianPdf.h 
                        include/quaternionEKF.h
                        include/quaternionEKFModule.h
                        include/quaternionEKFThread.h)


source_group("Source Files" FILES ${source_dir})
source_group("Header Files" FILES ${header_dir})

include_directories(SYSTEM ${YARP_INCLUDE_DIRS}
                           ${MATRIX_INCLUDE}
                           ${OROCOS_BFL_INCLUDEDIR}
                           ${Boost_INCLUDE_DIRS}
                           ${EIGEN3_INCLUDE_DIR}
                           ${PROJECT_SOURCE_DIR}/include
                           ${ctrlLib_INCLUDE_DIRS})

message("** Matrix include dirs are: " ${MATRIX_INCLUDE})
message("** Orocos-bfl include dirs are: " ${OROCOS_BFL_INCLUDEDIR})
message("** Boost include dirs are: " ${Boost_INCLUDE_DIRS})
message("** Boost libraries are: ${Boost_LIBRARIES}")

link_directories(${OROCOS_BFL_LIBRARY_DIRS})

message("-------------------- ${PROJECTNAME} ${source_dir} ${header_dir}")
add_executable(${PROJECTNAME} ${source_dir} ${header_dir})


target_link_libraries(${PROJECTNAME}
                      ${OROCOS_BFL_LIBRARIES}
	                  ${YARP_LIBRARIES}
	                  ${MATRIX_LIBS}
	                  ${Boost_LIBRARIES}
	                  ctrlLib)

if(WIN32)
INSTALL_TARGETS(/bin/Release ${PROJECTNAME})
else(WIN32)
    INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)

add_subdirectory(app)
