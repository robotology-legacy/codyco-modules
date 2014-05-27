# Copyright (C) 2010 RobotCub Consortium
# Author: Francesco Romano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(Eigen3 REQUIRED)
find_package(codycoCommons REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

FILE(GLOB folder_source ./*.cpp)
FILE(GLOB folder_header ./*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${codycoCommons_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

add_executable(codycoCommons-tests main.cpp)

target_link_libraries(codycoCommons-tests codyco_commons)

add_test(NAME test_codycoCommons-tests COMMAND codycoCommons-tests)
