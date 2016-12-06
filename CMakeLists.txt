# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


#### CMake Setup
cmake_minimum_required(VERSION 2.8.11)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/modules)

#### CoDyCo Setup
project(codyco-modules)

set(VARS_PREFIX ${PROJECT_NAME})
set(${PROJECT_NAME}_VERSION_MAJOR "0")
set(${PROJECT_NAME}_VERSION_MINOR "2")
set(${PROJECT_NAME}_VERSION_PATCH "0")
set(${PROJECT_NAME}_VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}")

find_package(YARP 2.3.68 REQUIRED)
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

# Force dynamic plugins. Must be set before
# yarp_configure_external_installation to ensure that codyco.ini is
# generated correctly
set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "CoDyCo is always built with dynamic plugins")
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco WITH_PLUGINS)
# Hide YARP_FORCE_DYNAMIC_PLUGINS from gui
# (yarp_configure_external_installation changes TYPE to BOOL)
set_property(CACHE YARP_FORCE_DYNAMIC_PLUGINS PROPERTY TYPE INTERNAL)

#### Find iCub
find_package(ICUB REQUIRED)

#### Set binary dir to ${CMAKE_BINARY_DIR}/bin
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
make_directory(${EXECUTABLE_OUTPUT_PATH})
mark_as_advanced(EXECUTABLE_OUTPUT_PATH)

#### Set library dir to ${CMAKE_BINARY_DIR}/lib
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
make_directory(${LIBRARY_OUTPUT_PATH})
mark_as_advanced(LIBRARY_OUTPUT_PATH)

### Print author warnings when a cmake_minimum_required version is higher then a given cmake version
if(NOT CMAKE_MINIMUM_REQUIRED_VERSION VERSION_LESS 2.8.12)
    # Just a reminder to remove some workarodunds  when we change cmake version
    message(AUTHOR_WARNING "CMake minimum required version greater than 2.8.12 . You can now get rid of the include_directories(SYSTEM ..) and substitute them with target_include_directories(... SYSTEM ...) .")
    message(AUTHOR_WARNING "CMake minimum required version greater than 2.8.12 . You can now get rid of the target_link_libraries(... LINK_PUBLIC ...) and substitute them with target_link_libraries(... PUBLIC ...) .")
endif()

if (CODYCO_BUILD_TESTS)
   include (CTest)
endif()

add_subdirectory(src)

# Export IDL libraries to simplify RPC communication
include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME} VARS_PREFIX ${VARS_PREFIX}
                                            VERSION ${${VARS_PREFIX}_VERSION}
                                            COMPATIBILITY SameMajorVersion
                                            TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)

#### add the "uninstall" target
include(AddUninstallTarget)
