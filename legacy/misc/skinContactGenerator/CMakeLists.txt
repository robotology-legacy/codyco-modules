CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME skinContactGenerator)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})


include_directories(${YARP_INCLUDE_DIRS} include)

include(YarpIDL)

set(generated_libs_dir "${CMAKE_CURRENT_SOURCE_DIR}")
yarp_idl_to_dir(skinContactGenerator.thrift ${generated_libs_dir})

# CMake has compiled the file during the configuration process
# so now we can include the generated .cmake file that lists all sources into "headers" and "sources" variables
include(skinContactGenerator_thrift.cmake)
include_directories(${generated_libs_dir}/include ${YARP_INCLUDE_DIRS})


include_directories(${YARP_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS})

add_executable(skinContactGenerator src/main.cpp src/skinContactGenerator_IDLServer.cpp include/skinContactGenerator_IDLServer.h)

target_link_libraries(skinContactGenerator  skinDynLib
                                            ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
