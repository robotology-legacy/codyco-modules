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
# The following PkgConfig is for searching OROCOS_BFL
find_package(PkgConfig)
pkg_check_modules(OROCOS_BFL REQUIRED orocos-bfl)

option(QUATERNION_EKF_USES_XSENS "Allows using an external XSens attached to your computer assuming that you have installed XSens drivers a priori." FALSE)

if(QUATERNION_EKF_USES_XSENS)
    find_package(XSens REQUIRED)
    CONFIGURE_FILE(./quaternionEKFconfig.h.in ${PROJECT_SOURCE_DIR}/include/quaternionEKFconfig.h)
endif()

if(QUATERNION_EKF_USES_XSENS)
    file(GLOB source_dir src/*.cpp)
    file(GLOB header_dir include/*.h)
elseif(QUATERNION_EKF_USES_XSENS)
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
endif(QUATERNION_EKF_USES_XSENS)

source_group("Source Files" FILES ${source_dir})
source_group("Header Files" FILES ${header_dir})

include_directories(SYSTEM ${YARP_INCLUDE_DIRS}
                           ${MATRIX_INCLUDE}
                           ${OROCOS_BFL_INCLUDEDIR}
                           ${Boost_INCLUDE_DIRS}
                           ${PROJECT_SOURCE_DIR}/include
                           ${ctrlLib_INCLUDE_DIRS})

if(QUATERNION_EKF_USES_XSENS)
    include_directories(SYSTEM ${XSENS_INCLUDE_DIR})
    message("** XSens include dirs are in: " ${XSENS_INCLUDE_DIR})
    message("** XSens all static libraries: ${XSENS_LIBRARIES}")
    #add_definitions(-DQUATERNION_EKF_USES_XSENS)
endif()

message("** Matrix include dirs are: " ${MATRIX_INCLUDE})
message("** Orocos-bfl include dirs are: " ${OROCOS_BFL_INCLUDEDIR})
message("** Boost include dirs are: " ${Boost_INCLUDE_DIRS})
message("** Boost libraries are: ${Boost_LIBRARIES}")


link_directories(${OROCOS_BFL_LIBRARY_DIRS})

add_executable(${PROJECTNAME} ${source_dir} ${header_dir})


target_link_libraries(${PROJECTNAME}
                      ${OROCOS_BFL_LIBRARIES}
	              ${YARP_LIBRARIES}
	              ${MATRIX_LIBS}
	              ${Boost_LIBRARIES}
	              ctrlLib)

if(QUATERNION_EKF_USES_XSENS)
    target_link_libraries(${PROJECTNAME}
                          ${XSENS_LIBRARIES})
endif(QUATERNION_EKF_USES_XSENS)


if(WIN32)
INSTALL_TARGETS(/bin/Release ${PROJECTNAME})
else(WIN32)
    INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)

add_subdirectory(app)
