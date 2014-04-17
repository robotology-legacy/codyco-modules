if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    unset(CODYCO_HAS_CHAR16)
    include(CheckCXXSourceCompiles)
    # SET(CMAKE_REQUIRED_FLAGS "-std=c++11")
    CHECK_CXX_SOURCE_COMPILES("
        #if defined(__STDC_UTF_16__) || (defined(_HAS_CHAR16_T_LANGUAGE_SUPPORT) && _HAS_CHAR16_T_LANGUAGE_SUPPORT)
        typedef char16_t CHAR16_T;
        #elif defined(_MSC_VER)
        typedef wchar_t CHAR16_T;
        #else
        typedef UINT16_T CHAR16_T;
        #endif
    
        int main() {
            CHAR16_T myVar;
            return 0;
        }
    
        "
    CODYCO_HAS_CHAR16)
    if (NOT CODYCO_HAS_CHAR16)
        add_definitions(-Dchar16_t=UINT16_T)
    else()
    endif(NOT CODYCO_HAS_CHAR16)
endif(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")

add_subdirectory(libraries)
#add_subdirectory(controllers)
