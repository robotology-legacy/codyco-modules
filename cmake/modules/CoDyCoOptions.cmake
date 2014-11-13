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
if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER "2.8.12")
    message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(NOT (CMAKE_VERSION VERSION_LESS 2.8.12))
    if(NOT MSVC)
        #add the option to disable RPATH
        set(CODYCOMODULES_DISABLE_RPATH FALSE CACHE BOOL "Disable RPATH installation")
        mark_as_advanced(CODYCOMODULES_DISABLE_RPATH)
    endif(NOT MSVC)

    #Configure RPATH
    set(CMAKE_MACOSX_RPATH 1) #enable RPATH on OSX. This also suppress warnings on CMake >= 3.0
    # when building, don't use the install RPATH already
    # (but later on when installing)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
    #build directory by default is built with RPATH
    set(CMAKE_SKIP_BUILD_RPATH  FALSE)

    #This is relative RPATH for libraries built in the same project
    #I assume that the directory is 
    # - install_dir/something for binaries
    # - install_dir/lib for libraries
    file(RELATIVE_PATH _rel_path "${CMAKE_INSTALL_PREFIX}/bin" "${CMAKE_INSTALL_PREFIX}/lib")
    if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(CMAKE_INSTALL_RPATH "@loader_path/${_rel_path}")
    else()
        set(CMAKE_INSTALL_RPATH "\$ORIGIN/${_rel_path}")
    endif()

    # the RPATH to be used when installing, but only if it's not a system directory (copied form CMake File. To be tested)
    list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
    if("${isSystemDir}" STREQUAL "-1")
       set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    endif("${isSystemDir}" STREQUAL "-1")

    # add the automatically determined parts of the RPATH
    # which point to directories outside the build tree to the install RPATH
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE) #very important!

    if(CODYCOMODULES_DISABLE_RPATH)
        #what to do? disable RPATH altogether or just revert to the default CMake configuration?
        #I revert to default
        unset(CMAKE_INSTALL_RPATH) #remove install rpath
        set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
    endif(CODYCOMODULES_DISABLE_RPATH)
endif()
#####end RPATH

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

if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER "2.8.10")
    message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(CMAKE_VERSION VERSION_GREATER 2.8.10)
    #define debug flag
    set_property(DIRECTORY APPEND PROPERTY
      COMPILE_DEFINITIONS $<$<CONFIG:Debug>:DEBUG=1>
    )
else()
    #define debug flag
    set_property(
        DIRECTORY
        APPEND PROPERTY COMPILE_DEFINITIONS_DEBUG DEBUG=1
    )
endif()

#### Option for building tests
option(CODYCO_BUILD_TESTS "Compile tests" FALSE)

#### Option for build wholeBodyReach module
option(CODYCO_BUILD_WHOLEBODYREACH "Compile the wholeBodyReach module" FALSE)

option(CODYCO_USES_EIGEN320 "Use Eigen 3.2" TRUE)
