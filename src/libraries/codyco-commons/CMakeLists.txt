# Copyright: (C) 2013 RobotCub Consortium
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

cmake_minimum_required( VERSION 2.8 FATAL_ERROR)

project(codyco_commons)

find_package(YARP)

# set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

find_package(Eigen3 REQUIRED)

set(SRCS    src/MathUtils.cpp
            src/LockGuard.cpp
            src/ControllerThread.cpp
            src/ModelParsing.cpp)

set(HEADERS  include/codyco/Commons.h
             include/codyco/Utils.h
             include/codyco/LockGuard.h
             include/codyco/MathUtils.h
             include/codyco/ControllerThread.h
             include/codyco/ModelParsing.h)

source_group("Source Files" FILES ${SRCS})
source_group("Header Files" FILES ${HEADERS})

include_directories(include/codyco
                    ${EIGEN3_INCLUDE_DIR}
                    ${YARP_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${SRCS} ${HEADERS})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

icub_export_library(${PROJECT_NAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                    DESTINATION include/codyco
                                    FILES ${HEADERS})

