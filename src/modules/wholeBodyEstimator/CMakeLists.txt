# Copyright (C) 2015 CoDyCo
# Author: Jorhabib Eljaik
# CopyPolicy: Released under the terms of the GNU GPL v2.0

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME wholeBodyEstimator)
project(${PROJECTNAME})

find_package(YARP                    REQUIRED)
find_package(iDynTree                REQUIRED)
find_package(wholeBodyInterface      REQUIRED)
find_package(yarpWholeBodyInterface  REQUIRED)
find_package(Eigen3                  REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(MATRIX REQUIRED)

# The following PkgConfig is for searching OROCOS_BFL
# Note: The way OROCOS_BFL is searched and so on might
# be necessary to get updated with more recent CMake directives.
if(CODYCO_USES_OROCOS_BFL_BERDY)
    find_package(PkgConfig)
    pkg_check_modules(OROCOS_BFL REQUIRED orocos-bfl)
endif()

file(GLOB source_dir    src/main.cpp
                        src/WholeBodyEstimatorModule.cpp
                        src/WholeBodyEstimatorThread.cpp
                        src/EstimatorsFactory.cpp
                        src/EstimatorsCreator.cpp
                        src/IDestructors.cpp
                        src/portsInterface.cpp
                        src/QuaternionEKF.cpp
                        src/LeggedOdometry.cpp
                        src/nonLinearAnalyticConditionalGaussian.cpp
                        src/nonLinearMeasurementGaussianPdf.cpp
                        src/DirectFiltering.cpp
                        src/floatingBase.cpp)
file(GLOB header_dir    include/WholeBodyEstimatorModule.h
                        include/WholeBodyEstimatorThread.h
                        include/IEstimator.h
                        include/EstimatorsFactory.h
                        include/EstimatorsCreator.h
                        include/EstimatorsCreatorImpl.h
                        include/constants.h
                        include/portsInterface.h
                        include/QuaternionEKF.h
                        include/LeggedOdometry.h
                        include/nonLinearAnalyticConditionalGaussian.h
                        include/nonLinearMeasurementGaussianPdf.h
                        include/DirectFiltering.h
                        include/floatingBase.h)

source_group("Source Files" FILES ${source_dir})
source_group("Header Files" FILES ${header_dir})

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR}
                           ${YARP_INCLUDE_DIRS}
                           ${MATRIX_INCLUDE}
                           ${OROCOS_BFL_INCLUDEDIR}
                           ${wholeBodyInterface_INCLUDE_DIRS}
                           ${yarpWholeBodyInterface_INCLUDE_DIRS}
                           ${iDynTree_INCLUDE_DIRS})

include_directories(include)

message("** wholeBodyEstimator says: ")
message("** Matrix include dirs are: " ${MATRIX_INCLUDE})
message("** Orocos-bfl include dirs are: " ${OROCOS_BFL_INCLUDEDIR})
message("** wholeBodyInterface include dirs are: " ${wholeBodyInterface_INCLUDE_DIRS})
message("** yarpWholeBodyInterface include dirs are:  " ${yarpWholeBodyInterface_INCLUDE_DIRS})
message("** iDynTree include dirs are:  " ${iDynTree_INCLUDE_DIRS})
message("** yarp libraries are: " ${YARP_LIBRARIES})

link_directories(${OROCOS_BFL_LIBRARY_DIRS})

message("${PROJECTNAME} ${source_dir} ${header_dir}")
add_executable(${PROJECTNAME} ${source_dir} ${header_dir})

if(CODYCO_USES_OROCOS_BFL_BERDY)
target_link_libraries(${PROJECTNAME}
                      ${OROCOS_BFL_LIBRARIES}
                      ${MATRIX_LIBS}
                      ${YARP_LIBRARIES}
                      ${wholeBodyInterface_LIBRARIES}
                      ${yarpWholeBodyInterface_LIBRARIES}
                      ${iDynTree_LIBRARIES})
else()
target_link_libraries(${PROJECTNAME}
                      ${YARP_LIBRARIES}
                      ${wholeBodyInterface_LIBRARIES}
                      ${yarpWholeBodyInterface_LIBRARIES}
                      ${iDynTree_LIBRARIES})
endif()

if(WIN32)
INSTALL_TARGETS(/bin/Release ${PROJECTNAME})
else(WIN32)
    INSTALL_TARGETS(/bin ${PROJECTNAME})
endif(WIN32)

add_subdirectory(app)
