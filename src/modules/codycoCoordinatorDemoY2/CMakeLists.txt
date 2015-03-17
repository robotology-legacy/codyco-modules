cmake_minimum_required(VERSION 2.8.11)

project(CodycoCoordinatorY2)

set(SOURCES src/main.cpp src/Coordinator.cpp)
set(HEADERS include/codyco/y2/Coordinator.h)

find_package(YARP REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)

include_directories(include/codyco/y2)

include_directories(SYSTEM ${YARP_INCLUDE_DIRS}
                           ${yarpWholeBodyInterface_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} 
                                      ${yarpWholeBodyInterface_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_subdirectory(app)
