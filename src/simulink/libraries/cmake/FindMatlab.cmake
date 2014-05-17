# Version slightly corrected to find latest Matlab versions. 
# Special thanks for Tom Lampert for fixes to make it work on OSX
# Mikhail Sirotenko (2012)
# - this module looks for Matlab
# Defines:
#  MATLAB_INCLUDE_DIR: include path for mex.h, engine.h
#  MATLAB_LIBRARIES:   required libraries: libmex, etc
#  MATLAB_MEX_LIBRARY: path to libmex.lib
#  MATLAB_MX_LIBRARY:  path to libmx.lib
#  MATLAB_ENG_LIBRARY: path to libeng.lib
#  MATLAB_MEXFILE_EXT: platform-specific extension for mex-files
#=============================================================================
# Copyright 2005-2009 Kitware, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# * Neither the names of Kitware, Inc., the Insight Software Consortium,
# nor the names of their contributors may be used to endorse or promote
# products derived from this software without specific prior written
# permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    # LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    # DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    # THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

        # Modified to properly find the libraries on Windows 32/64 bits by Jorhabib Eljaik,
        # Istituto Italiano di Tecnologia, RBCS Department. Genoa, Italy.
        # This module should find from Matlab 7.1 R14SP3 to Matlab 8 R2012b
        #=============================================================================

        SET(MATLAB_FOUND 0)

        IF(NOT MATLAB_ROOT)
            message("MATLAB_ROOT = TRUE")
            SET(MATLAB_MEX_LIBRARY MATLAB_MEX_LIBRARY-NOTFOUND)
            SET(MATLAB_MX_LIBRARY MATLAB_MX_LIBRARY-NOTFOUND)
            SET(MATLAB_ENG_LIBRARY MATLAB_ENG_LIBRARY-NOTFOUND)
            SET(MATLAB_ARCH MATLAB_ARCH-NOTFOUND)
            IF(WIN32)
                #Find Matlab root. Suppose the Matlab version is between 7.1 and 8.15
                #SET(MATLAB_ROOT "")
                SET(MATLAB_INCLUDE_DIR "")
                SET(_MATLAB_INCLUDE_DIR "")		
                SET(REGISTRY_KEY_TMPL "[HKEY_LOCAL_MACHINE\\SOFTWARE\\MathWorks\\MATLAB\\TMPL_MAJOR.TMPL_MINOR;MATLABROOT]")
                FOREACH(MATLAB_MAJOR RANGE 7 8)
                    FOREACH(MATLAB_MINOR RANGE 15)
                        #STRING(REPLACE "TMPL_MINOR" "${MATLAB_MINOR}" REGISTRY_KEY_OUTP ${REGISTRY_KEY_TMPL})
                        STRING(REPLACE "TMPL_MINOR" "${MATLAB_MINOR}" REGISTRY_KEY_OUTP ${REGISTRY_KEY_TMPL})
                        STRING(REPLACE "TMPL_MAJOR" "${MATLAB_MAJOR}" REGISTRY_KEY_OUTP ${REGISTRY_KEY_OUTP})
                        GET_FILENAME_COMPONENT(_MATLAB_ROOT ${REGISTRY_KEY_OUTP} ABSOLUTE)
                        IF(NOT ${_MATLAB_ROOT} STREQUAL "/registry")
                            SET(MATLAB_ROOT ${_MATLAB_ROOT})
                            MESSAGE("MATLAB ROOT IS " ${MATLAB_ROOT})
                        ENDIF()
                    ENDFOREACH()
                ENDFOREACH()

                IF(NOT ${CMAKE_GENERATOR} MATCHES "Visual Studio")
                    IF(MATLAB_FIND_REQUIRED)
                        MESSAGE(FATAL_ERROR "Generator not compatible: ${CMAKE_GENERATOR}")
                    ENDIF(MATLAB_FIND_REQUIRED)
                ENDIF(NOT ${CMAKE_GENERATOR} MATCHES "Visual Studio")
	  	  
                EXECUTE_PROCESS(COMMAND "${MATLAB_ROOT}/bin/mexext.bat" OUTPUT_VARIABLE MATLAB_MEXFILE_EXT OUTPUT_STRIP_TRAILING_WHITESPACE)
      
                FIND_LIBRARY(MATLAB_MEX_LIBRARY
                    libmex
                    HINTS "${MATLAB_ROOT}/extern/lib/win64/microsoft/"
                    "${MATLAB_ROOT}/extern/lib/win32/microsoft/"
                    NO_DEFAULT_PATH
                )

                FIND_LIBRARY(MATLAB_MX_LIBRARY
                    libmx
                    HINTS "${MATLAB_ROOT}/extern/lib/win64/microsoft/"
                    "${MATLAB_ROOT}/extern/lib/win32/microsoft/"
                    NO_DEFAULT_PATH
                )
        
                FIND_LIBRARY(MATLAB_ENG_LIBRARY
                    libeng
                    HINTS "${MATLAB_ROOT}/extern/lib/win64/microsoft/"
                    "${MATLAB_ROOT}/extern/lib/win32/microsoft/"
                    NO_DEFAULT_PATH
                )
		
                message("MATLAB_MEX_LIBRARY FOUND IN = ${MATLAB_MEX_LIBRARY}")
                message("MATLAB_MX_LIBRARY  FOUND IN = ${MATLAB_MX_LIBRARY}")
                message("MATLAB_ENG_LIBRARY FOUND IN = ${MATLAB_ENG_LIBRARY}")
		
                # This seems to be the most reliable method to date.
                message("MATLAB_MEXFILE_EXT = ${MATLAB_MEXFILE_EXT}")
                STRING(REGEX MATCH ..$ MATLAB_ARCH MATLAB_MEXFILE_EXT 64)
                message("MATLAB VERSION IS ${MATLAB_ARCH}bits")
		
            ELSE( WIN32 )
                SET(_MATLAB_ROOT_LST
                    $ENV{MATLABDIR}
                    $ENV{MATLAB_DIR}
                    /usr/local/matlab-7sp1/
                    /opt/matlab-7sp1/
                    $ENV{HOME}/matlab-7sp1/
                    $ENV{HOME}/redhat-matlab/
                    /usr/local/MATLAB/R2013a/
                    /usr/local/MATLAB/R2012b/
                    /usr/local/MATLAB/R2012a/
                    /usr/local/MATLAB/R2011b/
                    /usr/local/MATLAB/R2011a/
                    /usr/local/MATLAB/R2010bSP1/
                    /usr/local/MATLAB/R2010b/
                    /usr/local/MATLAB/R2010a/
                    /usr/local/MATLAB/R2009bSP1/
                    /usr/local/MATLAB/R2009b/
                    /usr/local/MATLAB/R2009a/
                    /usr/local/MATLAB/R2008b/
                    /usr/local/MATLAB/R2008a/
                    /Applications/MATLAB_R2012b.app/
                    /Applications/MATLAB_R2012a.app/
                    /Applications/MATLAB_R2011b.app/
                    /Applications/MATLAB_R2011a.app/
                    /Applications/MATLAB_R2010bSP1.app/
                    /Applications/MATLAB_R2010b.app/
                    /Applications/MATLAB_R2010a.app/
                    /Applications/MATLAB_R2009bSP1.app/
                    /Applications/MATLAB_R2009b.app/
                    /Applications/MATLAB_R2009a.app/
                    /Applications/MATLAB_R2008b.app/
                    /Applications/MATLAB_R2008a.app/
                )
                #SET (LOOP_VAR "")
                SET(_MATLAB_ROOT "bin-NOTFOUND")
                FOREACH(LOOP_VAR ${_MATLAB_ROOT_LST})
                    FIND_PATH(_MATLAB_ROOT "bin" ${LOOP_VAR} NO_DEFAULT_PATH)
                    IF(_MATLAB_ROOT)			
                        SET(MATLAB_ROOT ${_MATLAB_ROOT})
                        BREAK()
                    ENDIF()
                ENDFOREACH()
		
                EXECUTE_PROCESS(COMMAND "${MATLAB_ROOT}/bin/mexext" OUTPUT_VARIABLE MATLAB_MEXFILE_EXT OUTPUT_STRIP_TRAILING_WHITESPACE)
                message("MATLAB_MEXFILE_EXT = ${MATLAB_MEXFILE_EXT}")
                # The following regual expression matches the last two digits of the MEX extension given in MATLAB_MEXFILE_EXT
                # with number 64. The reason to do this can be understood by first recalling that the MEX extensions for the different
                # operating systems are as follows: 
                # mexglx: 	Linux-based OS 32bits
                # mexa64: 	Linux-based OS 64bits
                # mexmaci64:	MAC OS 64bits
                # mexw32:	Windows OS 32bits
                # mexw64: 	Windows OS 64bits
                # As you can see, all the previous extensions finish with a number corresponding to the OS version, except for
                # mexglx. The regular expresion 64$ is used to match the number 64 at the end of the retrieved extension. If it does
                    # not match 64 it will either be empty (as it would happen for mexglx or 32 bits).
                    STRING(REGEX MATCH 64$ MATLAB_ARCH ${MATLAB_MEXFILE_EXT})

                    IF("${MATLAB_ARCH}" STREQUAL "64")
                        SET(_MATLAB_LIB_DIR "bin-NOTFOUND")

                        # Intel64 (OSX):
                        IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
                            FIND_PATH(_MATLAB_LIB_DIR "maci64" ${MATLAB_ROOT}/bin/ NO_DEFAULT_PATH)
                            IF(_MATLAB_LIB_DIR)
                                message("CONFIGURING FOR MAC")
                                SET(MATLAB_LIB_DIR ${_MATLAB_LIB_DIR}/maci64)
                            ENDIF()
                        ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

                        # AMD64 (Linux):    
                        IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
                            FIND_PATH(_MATLAB_LIB_DIR "glnxa64" ${MATLAB_ROOT}/bin/ NO_DEFAULT_PATH)
                            IF(_MATLAB_LIB_DIR)			
                                message("CONFIGURING FOR LINUX 64 BITS")
                                SET(MATLAB_LIB_DIR ${_MATLAB_LIB_DIR}/glnxa64)
                            ENDIF()
                        ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	      

                    ELSE("${MATLAB_ARCH}" STREQUAL "64")
                        message("CONFIGURING FOR MATLAB 32 BITS")
                        # Regular x86
                        SET(MATLAB_LIB_DIR "${MATLAB_ROOT}/bin/glnx86/")
                    ENDIF("${MATLAB_ARCH}" STREQUAL "64")
	    
                    FIND_LIBRARY(MATLAB_MEX_LIBRARY
                        mex
                        ${MATLAB_LIB_DIR}
                        NO_DEFAULT_PATH
                    )
                    FIND_LIBRARY(MATLAB_MX_LIBRARY
                        mx
                        ${MATLAB_LIB_DIR}
                        NO_DEFAULT_PATH
                    )
                    FIND_LIBRARY(MATLAB_ENG_LIBRARY
                        eng
                        ${MATLAB_LIB_DIR}
                        NO_DEFAULT_PATH
                    )
		
                    message("MATLAB_MEX_LIBRARY FOUND IN = ${MATLAB_MEX_LIBRARY}")
                    message("MATLAB_MX_LIBRARY  FOUND IN = ${MATLAB_MX_LIBRARY}")
                    message("MATLAB_ENG_LIBRARY FOUND IN = ${MATLAB_ENG_LIBRARY}")

                ENDIF(WIN32)
            ENDIF(NOT MATLAB_ROOT)


            SET(MATLAB_INCLUDE_DIR "${MATLAB_ROOT}/extern/include/")

            # This is common to UNIX and Win32:
            SET(MATLAB_LIBRARIES
                ${MATLAB_MEX_LIBRARY}
                ${MATLAB_MX_LIBRARY}
                ${MATLAB_ENG_LIBRARY}
            )

            IF(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)
                SET(MATLAB_FOUND 1)
            ENDIF(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)

            MARK_AS_ADVANCED(
                MATLAB_LIBRARIES
                MATLAB_MEX_LIBRARY
                MATLAB_MX_LIBRARY
                MATLAB_ENG_LIBRARY
                MATLAB_INCLUDE_DIR
                MATLAB_FOUND
                MATLAB_ROOT
                MATLAB_MEXFILE_EXT
                MATLAB_ARCH
            )

