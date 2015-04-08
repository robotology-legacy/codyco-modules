# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

PROJECT(torqueGraspManager)

find_package(ICUB)
find_package(YARP)
find_package(GSL)

SET(SOURCES src/main.cpp src/ManagerModule.cpp src/ManagerThread.cpp src/Utilities.cpp)
SET(HEADERS include/codyco/ManagerModule.h include/codyco/ManagerThread.h include/codyco/Utilities.h)

include_directories(include/codyco)

include_directories(SYSTEM ${ctrlLib_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} ctrlLib
                                     icubmod
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_subdirectory(app)
