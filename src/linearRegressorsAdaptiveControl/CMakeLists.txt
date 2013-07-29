cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME linearRegressorsAdaptiveControl)
PROJECT(${PROJECTNAME})

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
message(STATUS "CMakeModulePath" ${CMAKE_MODULE_PATH})

find_package(YARP)
find_package(ICUB)
find_package(Eigen3)
find_package(Orocos-KDL REQUIRED)
find_package(kdl_codyco REQUIRED)


IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
     IF(CMAKE_COMPILER_IS_GNUCC)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
     ENDIF(CMAKE_COMPILER_IS_GNUCC)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

#optionally: use cmake find scripts provided by YARP and iCub

set(CMAKE_INSTALL_PREFIX "$ENV{ICUB_DIR}" CACHE PATH "Install path prefix, pre-appended onto install directory." FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall ${ICUB_LINK_FLAGS}")


include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${iDynTree_INCLUDE_DIRS} 					${iDyn_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/linearRegressorsAdaptiveControl/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})
ADD_EXECUTABLE(${PROJECTNAME}  ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES}  ${iCub_LIBRARIES} ${iDyn_LIBRARIES} wbi wbiy iDyn iDynTree ${Orocos-KDL_LIBRARIES} ${kdl_codyco_LIBRARIES})


if(WIN32)
	INSTALL(TARGETS ${PROJECTNAME} RUNTIME DESTINATION bin/release)
	SET_TARGET_PROPERTIES(${PROJECTNAME}  PROPERTIES COMPILE_FLAGS /wd4996)
else(WIN32)
	INSTALL(TARGETS ${PROJECTNAME} RUNTIME DESTINATION bin)
endif(WIN32)


