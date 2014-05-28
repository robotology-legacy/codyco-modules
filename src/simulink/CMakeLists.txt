
#On new versions of Clang, MATLAB requires C++11.
#I enable it on all Clangs
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
        if (${CMAKE_GENERATOR} MATCHES "Xcode")
            #this should set explictly the option in xcode. Unfortunately it does not work.
            set(XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "C++11")            
        endif(${CMAKE_GENERATOR} MATCHES "Xcode")
    endif(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_subdirectory(libraries)
#add_subdirectory(controllers)
