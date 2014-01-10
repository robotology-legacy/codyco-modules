# Copyright (C) 2013 CoDyCo
# Author: Jorhabib Eljaik
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
SET(PROJECTNAME yWrite)
PROJECT(${PROJECTNAME})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(YARP)
find_package(Matlab REQUIRED)

if(MATLAB_FOUND)
	message(STATUS "Matlab found")
else()
	message(STATUS "Matlab not found")
	set(MATLAB_LIBRARIES 
	  ${MATLAB_MEX_LIBRARY}
	  ${MATLAB_MX_LIBRARY}
	  ${MATLAB_ENG_LIBRARY}
	)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories("${MATLAB_ROOT}/simulink/include")
include_directories("${MATLAB_ROOT}/extern/include")
include_directories(${MATLAB_INCLUDE_DIR})
include_directories("${YARP_INCLUDE_DIRS}")
include_directories(${PROJECT_SOURCE_DIR}/include)

add_definitions(-DMATLAB_MEX_FILE)

set(SFUNCTION_SUFFIX ${MATLAB_MEXFILE_EXT})

link_directories(${CMAKE_BINARY_DIR})
#link_directories("${MATLAB_PATH}/bin/glnxa86")
#link_directories(${MATLAB_LIBRARIES})

add_library(${PROJECTNAME} SHARED ${folder_source} ${folder_header})

set(CMAKE_CXX_FLAGS "-fpermissive")

if(NOT "${MATLAB_ROOT}" STREQUAL "")
	# if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	# message("SIZEOF_VOID_P = ${CMAKE_SIZEOF_VOID_P}")
                        # set_target_properties(${PROJECTNAME} PROPERTIES SUFFIX .mexa64)
                        # set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_FLAGS "-I$ENV{MATLAB_ROOT} -O -pthread -shared -Wl,--version-script,$ENV{MATLAB_ROOT}/extern/lib/glnxa64/mexFunction.map -Wl,--no-undefined -Wl,-rpath-link,$ENV{MATLAB_ROOT}/bin/glnxa64 -L$ENV{MATLAB_ROOT}/bin/glnxa64 -lmx -lmex -lmat -lm -lstdc++")
	# else()
                        # set_target_properties(${PROJECTNAME} PROPERTIES SUFFIX .mexglx
										   # PREFIX "")
                        # set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_FLAGS "-I$ENV{MATLAB_ROOT} -O -pthread -shared -m32 -Wl,--version-script,$ENV{MATLAB_ROOT}/extern/lib/glnx86/mexFunction.map -Wl,--no-undefined -Wl,-rpath-link,$ENV{MATLAB_ROOT}/bin/glnx86 -L$ENV{MATLAB_ROOT}/bin/glnx86 -lmx -lmex -lmat -lm -lstdc++")
	# endif()		
	
	
	
	set_target_properties(${PROJECTNAME} PROPERTIES SUFFIX .${SFUNCTION_SUFFIX})
	set_target_properties(${PROJECTNAME} PROPERTIES PREFIX "")

	# if("${MATLAB_ARCH}" STREQUAL "64")
	# 	if(WIN32)
	# 		message("Flags have been passed to compiler for Windows 64bits")
	# 		set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_FLAGS "-I$ENV{MATLAB_ROOT} -O -pthread -shared -Wl,--version-script, -Wl,--no-undefined -Wl,-rpath-link,$ENV{MATLAB_ROOT}/bin/win64 -L$ENV{MATLAB_ROOT}/bin/win64 -lmx -lmex -lmat -lm -lstdc++")
	# 	elseif(WIN32)
	# 		set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_FLAGS "-I$ENV{MATLAB_ROOT} -O -pthread -shared -Wl,--version-script,$ENV{MATLAB_ROOT}/extern/lib/glnxa64/mexFunction.map -Wl,--no-undefined -Wl,-rpath-link,$ENV{MATLAB_ROOT}/bin/glnxa64 -L$ENV{MATLAB_ROOT}/bin/glnxa64 -lmx -lmex -lmat -lm -lstdc++")
	# 	endif(WIN32)
	# else()
	# 	set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_FLAGS "-I$ENV{MATLAB_ROOT} -O -pthread -shared -m32 -Wl,--version-script,$ENV{MATLAB_ROOT}/extern/lib/glnx86/mexFunction.map -Wl,--no-undefined -Wl,-rpath-link,$ENV{MATLAB_ROOT}/bin/glnx86 -L$ENV{MATLAB_ROOT}/bin/glnx86 -lmx -lmex -lmat -lm -lstdc++")
	# endif()

	
	
	
        target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${MATLAB_LIBRARIES})
        install(TARGETS ${PROJECTNAME} DESTINATION mex)
endif(NOT "${MATLAB_ROOT}" STREQUAL "")
















