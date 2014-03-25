if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    INCLUDE (CheckCXXSourceCompiles)
    CHECK_CXX_SOURCE_COMPILES("
        #if defined(__STDC_UTF_16__) || (defined(_HAS_CHAR16_T_LANGUAGE_SUPPORT) && _HAS_CHAR16_T_LANGUAGE_SUPPORT)
        typedef char16_t CHAR16_T;
        #elif defined(_MSC_VER)
        typedef wchar_t CHAR16_T;
        #else
        typedef UINT16_T CHAR16_T;
        #endif

        int main() {
            return 0;
        }

        "
    HAS_CHAR16_T_TYPE)
    if (NOT HAS_CHAR16_T_TYPE)
        add_definitions(-Dchar16_t=uint16_t)
    endif(NOT HAS_CHAR16_T_TYPE)
endif(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")

add_subdirectory(libraries)
#add_subdirectory(controllers)
