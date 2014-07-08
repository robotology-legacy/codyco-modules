# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


#### CMake Setup
cmake_minimum_required(VERSION 2.8.7)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

#### CoDyCo Setup
project(CoDyCo)

set(CODYCO_VERSION_MAJOR "0")
set(CODYCO_VERSION_MINOR "1")
set(CODYCO_VERSION_PATCH "0")
set(CODYCO_VERSION "${CODYCO_VERSION_MAJOR}.${CODYCO_VERSION_MINOR}.${CODYCO_VERSION_PATCH}")

find_package(YARP 2.3.61 REQUIRED)
message(STATUS "YARP is version: ${YARP_VERSION}")

if (YARP_HAS_MATH_LIB)
     set(ICUB_HAS_YARP TRUE CACHE BOOL "" FORCE)
     message(STATUS "found libYARP_math")
else(YARP_HAS_MATH_LIB)
     set(ICUB_HAS_YARP FALSE CACHE BOOL "" FORCE)
     message(FATAL_ERROR "YARP was found, but no libYARP_math was detected, please recompile yarp")
endif(YARP_HAS_MATH_LIB)

add_definitions(${YARP_DEFINES})   #this contains also -D_REENTRANT

# include YARP CMake modules
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})

#### find dependencies and set variables
include(CoDyCoFindDependencies)

# Import common options (disable flags in windows, rpath, etc..)
include(CoDyCoOptions)

# new "data" installation - available with yarp2.4
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

#### Find iCub

find_package(ICUB REQUIRED)

# include iCub CMake modules
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

# add functionalities like icub_add_library/icub_add_executable
include(iCubHelpers)

#### Set binary dir to ${CMAKE_BINARY_DIR}/bin
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
make_directory(${EXECUTABLE_OUTPUT_PATH})
message(STATUS "Executables go to ${EXECUTABLE_OUTPUT_PATH}")
mark_as_advanced(EXECUTABLE_OUTPUT_PATH)

#### Set library dir to ${CMAKE_BINARY_DIR}/lib
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
make_directory(${LIBRARY_OUTPUT_PATH})
message(STATUS "Libraries go to ${LIBRARY_OUTPUT_PATH}")
mark_as_advanced(LIBRARY_OUTPUT_PATH)

### enable testing
option(CODYCO_DASHBOARD_SUBMIT "Submit compile tests to cdash" FALSE)
option(CODYCO_BUILD_TESTS "Compile tests" FALSE)
option(ICUB_USE_XDE "Experimental ISIR modules based on XDE" FALSE)
option(CODYCO_USES_WBI_TOOLBOX "Whole Body Interface Toolbox - Simulink library" FALSE)

if (CODYCO_DASHBOARD_SUBMIT OR CODYCO_BUILD_TESTS)
   include (CTest)
endif()

add_subdirectory(src)

#### add the "uninstall" target
include(AddUninstallTarget)
