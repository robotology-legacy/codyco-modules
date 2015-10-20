#  Robot Testing Framework (RTF)
#  Copyright: (C) 2015 Ali Paikan
#  Authors: Ali Paikan <ali.paikan@gmail.com>
#
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

cmake_minimum_required(VERSION 2.8.9)

# set the project name
set(PROJECTNAME matlabTorqueBalancingTest)
project(${PROJECTNAME})

# add the required cmake packages
find_package(RTF REQUIRED)
find_package(RTF COMPONENTS DLL YARP REQUIRED)
find_package(YARP)

# add include directories
include_directories(${CMAKE_SOURCE_DIR}
                    ${RTF_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

# add required libraries
link_libraries(${RTF_LIBRARIES}
               ${YARP_LIBRARIES})

# add the source codes to build the plugin library
set(CMAKE_SHARED_MODULE_PREFIX     "")
add_library(${PROJECTNAME} MODULE matlabTorqueBalancingTest.h
                                  matlabTorqueBalancingTest.cpp)

set_target_properties(${PROJECTNAME} PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../)

# set the installation options
install(TARGETS ${PROJECTNAME}
        EXPORT ${PROJECTNAME}
        COMPONENT runtime
        LIBRARY DESTINATION lib)

