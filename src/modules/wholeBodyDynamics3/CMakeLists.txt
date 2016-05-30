# Copyright: (C) 2016 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU LGPL v2.1+.

project(wholeBodyDynamics3)

source_group("Source Files" FILES main.cpp)

add_executable(${PROJECT_NAME} main.cpp)

include_directories(${YARP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_subdirectory(app)



