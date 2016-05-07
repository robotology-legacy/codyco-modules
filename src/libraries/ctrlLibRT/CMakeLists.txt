# Copyright (C) 2016 Istituto Italiano di Tecnologia  iCub Facility
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.11)

find_package(Eigen3 REQUIRED)

project(ctrlLibRT)

set(${PROJECT_NAME}_HDRS include/${PROJECT_NAME}/filters.h)

set(${PROJECT_NAME}_SRCS src/filters.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_HDRS} ${${PROJECT_NAME}_SRCS})

target_include_directories(${PROJECT_NAME} PUBLIC
                                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>")
target_include_directories(${PROJECT_NAME} PUBLIC  ${YARP_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} PROPERTY PUBLIC_HEADER ${${PROJECT_NAME}_HELPERS_SRCS})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

