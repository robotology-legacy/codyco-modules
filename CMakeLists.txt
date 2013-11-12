# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


#### CMake Setup
cmake_minimum_required(VERSION 2.6.4)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)



#### CoDyCo Setup
project(CoDyCo)

set(CODYCO_VERSION_MAJOR "0")
set(CODYCO_VERSION_MINOR "0")
set(CODYCO_VERSION_PATCH "1")
set(CODYCO_VERSION "${CODYCO_VERSION_MAJOR}.${CODYCO_VERSION_MINOR}.${CODYCO_VERSION_PATCH}")




#### Find YARP, require that YARP version is at least YARP_REQVERSION
set(YARP_REQVERSION_MAJOR "2")
set(YARP_REQVERSION_MINOR "3")
set(YARP_REQVERSION_PATCH "20")
set(YARP_REQVERSION_TWEAK "")
if(YARP_REQVERSION_TWEAK)
    set(YARP_REQVERSION ${YARP_REQVERSION_MAJOR}.${YARP_REQVERSION_MINOR}.${YARP_REQVERSION_PATCH}.${YARP_REQVERSION_TWEAK})
else()
    set(YARP_REQVERSION ${YARP_REQVERSION_MAJOR}.${YARP_REQVERSION_MINOR}.${YARP_REQVERSION_PATCH})
endif()


find_package(YARP REQUIRED)

message(STATUS "YARP is version: ${YARP_VERSION}")

if(YARP_VERSION VERSION_LESS YARP_REQVERSION)
    message(FATAL_ERROR "YARP version is not compatible, please update YARP")
else()
    message(STATUS "found compatible YARP version")
endif()

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




#### Find iCub

find_package(ICUB REQUIRED)

if(ICUB_VERSION VERSION_LESS ICUB_REQVERSION)
    message(FATAL_ERROR "iCub version is not compatible, please update iCub")
else()
    message(STATUS "found compatible iCub version")
endif()

# include iCub CMake modules
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})


# add functionalities like icub_add_library/icub_add_executable
include(iCubHelpers)

# Import common options (disable flags in windows, rpath, etc..)
include(CoDyCoOptions)




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




#### find dependencies and set variables
include(CoDyCoFindDependencies)




### enable testing
option(CODYCO_DASHBOARD_SUBMIT "Submit compile tests to cdash" FALSE)
option(ICUB_USE_XDE "Experimental ISIR modules based on XDE" FALSE)

if (CODYCO_DASHBOARD_SUBMIT)
   include (CTest)
endif()




#### add the "uninstall" target
configure_file(
  "${CMAKE_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_BINARY_DIR}/cmake_uninstall.cmake"
  @ONLY)

add_custom_target(uninstall_CoDyCo
  "${CMAKE_COMMAND}" -P "${CMAKE_BINARY_DIR}/icub-config-uninstall.cmake")

#### if you want to compile iDynTree you have to have the dependencies
find_package(Eigen3)
find_package(Orocos-KDL REQUIRED)
find_package(kdl_codyco REQUIRED)


#### Add external modules
#add_subdirectory(extern)

add_subdirectory(src)
add_subdirectory(doc)
