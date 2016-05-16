cmake_minimum_required(VERSION 2.8)

project(iCubParis02_data_analysis)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

#### Find kdl_codyco, require that kdl_codyco version is at least kdl_codyco_REQVERSION

find_package(kdl_codyco REQUIRED)
find_package(YARP REQUIRED)
find_file(ICUB REQUIRED)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")      


include_directories(${orocos_kdl_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} ${kdl_format_io_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${skinDynLib_INCLUDE_DIRS})

add_executable(iCubParis02_data_analysis iCubParis02_data_analysis.cpp multitaskSVDLinearEstimator.cpp multitaskRecursiveLinearEstimator.cpp)
add_executable(iCubParis02_simple_analysis iCubParis02_simple_analysis.cpp multitaskSVDLinearEstimator.cpp multitaskRecursiveLinearEstimator.cpp)

target_link_libraries(iCubParis02_data_analysis ${orocos_kdl_LIBRARIES} kdl_codyco kdl_format_io ${iDynTree_LIBRARIES} iDyn skinDynLib iDynTree ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
target_link_libraries(iCubParis02_simple_analysis ${orocos_kdl_LIBRARIES} kdl_codyco kdl_format_io ${iDynTree_LIBRARIES} iDyn skinDynLib iDynTree ${YARP_LIBRARIES} ${ICUB_LIBRARIES})


INSTALL(TARGETS iCubParis02_data_analysis DESTINATION bin)
INSTALL(TARGETS iCubParis02_simple_analysis DESTINATION bin)
