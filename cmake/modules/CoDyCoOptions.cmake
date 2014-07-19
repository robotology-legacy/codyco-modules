# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


#### Options

## URDF file format support
option(CODYCO_USES_URDFDOM "Enable support for URDF input in iDynTree" TRUE)
if(CODYCO_USES_URDFDOM)
    add_definitions(-DCODYCO_USES_URDFDOM)
endif()

# SET(CODYCO_TRAVIS_CI FALSE CACHE BOOL "Set if build is done with Travis-CI flags")
OPTION(CODYCO_TRAVIS_CI "Set if build is done with Travis-CI flags" FALSE)

if(MSVC)
    MESSAGE(STATUS "Running on windows")

    # ACE uses a bunch of functions MSVC warns about.
    # The warnings make sense in general, but not in this case.
    # this gets rids of deprecated unsafe crt functions
    add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    # this gets rid of warning about deprecated POSIX names
    add_definitions(-D_CRT_NONSTDC_NO_DEPRECATE)
    # Traditionally, we add "d" postfix to debug libraries

    # Trying to disable: warning C4355: 'this' : used ...
    # with no luck.
    ##add_definitions("/wd4355")
    ##set(CMAKE_CXX_FLAGS "/wd4355 ${CMAKE_CXX_FLAGS}")

    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, recommanded options are: Debug or Release" FORCE)
    endif()
    set(CODYCO_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CODYCO_BUILD_TYPES})
endif()


#### Settings for rpath

if(NOT MSVC)
    set(CODYCO_INSTALL_WITH_RPATH FALSE CACHE BOOL "Set an rpath after installing the executables")
    #mark_as_advanced(CODYCO_ENABLE_FORCE_RPATH)
endif(NOT MSVC)

if(CODYCO_INSTALL_WITH_RPATH)
    # when building, don't use the install RPATH already
    # (but later on when installing), this tells cmake to relink
    # at install, so in-tree binaries have correct rpath
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

    set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif (CODYCO_INSTALL_WITH_RPATH)


#### Settings for shared libraries

option(CODYCO_SHARED_LIBRARY "Compile shared libraries rather than static libraries" FALSE)
if(CODYCO_SHARED_LIBRARY)
    set(BUILD_SHARED_LIBS ON)
endif()

#setting options specific for OS X
#THIS should solve issues on building on OS X depending current system and Orocos KDL version
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    option(CODYCO_OSX_STDLIB "Compile CoDyCo by explicitly linking to the old GNU stdc++ library instead of the default one" FALSE)
    if (OROCOS_KDL_OLDVERSION)
        set(CODYCO_OSX_STDLIB TRUE)
    endif()

    #if needs GNU stdc++ AND 
        message("System: ${CMAKE_SYSTEM}")
        message("Version: ${CMAKE_SYSTEM_VERSION}")
        
    if (CODYCO_OSX_STDLIB)
        if (${CMAKE_GENERATOR} MATCHES "Xcode")
            MESSAGE("Xcode generator: setting standard libraries to libstdc++")
            SET(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libstdc++")
        else()
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
            SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
        endif()
    endif(CODYCO_OSX_STDLIB)
endif()

#setting debug options
if(MSVC)
    ###
else()
    ##Other systems
    if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Weverything -pedantic -Wnon-virtual-dtor -Woverloaded-virtual")
        #disable padding alignment warnings. Cast align is more subtle. On X86 it should not create any problem but for different architecture we should handle this warning better.
        SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wno-padded -Wno-cast-align")
        if (CODYCO_TRAVIS_CI)
            #disable documentation warnings and sign comparison. This is for Travis-CI
            MESSAGE(STATUS "Disabling some warning for Travis-CI")
            SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wno-documentation -Wno-documentation-unknown-command -Wno-sign-conversion")
        endif()
        MESSAGE(STATUS "Clang compiler - Debug configuration flags: -Weverything -pedantic -Wnon-virtual-dtor -Woverloaded-virtual")
    elseif(${CMAKE_COMPILER_IS_GNUCC})
        SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra")
        if (NOT CODYCO_TRAVIS_CI)
            MESSAGE(STATUS "Gcc compiler - Debug configuration flags: -Wall -Wextra -pedantic -Weffc++ -Woverloaded-virtual")
            SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -pedantic -Weffc++ -Woverloaded-virtual")
        endif()
    endif()
endif()

#define debug flag
# SET(COMPILE_DEFINITIONS_DEBUG "${COMPILE_DEFINITIONS_DEBUG};DEBUG=1")
# add_definitions(-DDEBUG=1)
set_property(
    DIRECTORY
    APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG=1
)

#### Option for building tests
option(CODYCO_BUILD_TESTS "Compile tests" FALSE)

#### Option for build wholeBodyReach module
option(CODYCO_BUILD_WHOLEBODYREACH "Compile the wholeBodyReach module" FALSE)
