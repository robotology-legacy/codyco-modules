# Copyright: (C) 2014 Istituto Italiano di Tecnologia
# Authors: Silvio 
# CopyPolicy: Released under the terms of the GNU GPL v2.0+.


cmake_minimum_required(VERSION 2.8)
ENABLE_LANGUAGE(CXX)

find_package(YARP REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(${YARP_DEFINES})

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

include(YarpPlugin)

BEGIN_PLUGIN_LIBRARY(codycomod)

add_subdirectory(jointTorqueControl)

END_PLUGIN_LIBRARY(codycomod)

ADD_PLUGIN_LIBRARY_EXECUTABLE(codycomoddev codycomod)
target_link_libraries(codycomoddev ${YARP_LIBRARIES})

install(TARGETS codycomoddev DESTINATION bin)
install(TARGETS codycomod EXPORT codycoMod 
		LIBRARY DESTINATION lib COMPONENT shlib
		ARCHIVE DESTINATION lib  
		PUBLIC_HEADER DESTINATION include/codycomod COMPONENT dev)
