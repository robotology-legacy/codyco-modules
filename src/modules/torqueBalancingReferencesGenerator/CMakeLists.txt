cmake_minimum_required(VERSION 2.8.12)

project(torqueBalancingReferencesGenerator)

find_package(YARP REQUIRED)
find_package(yarpWholeBodyInterface REQUIRED)
find_package(iDynTree REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${yarpWholeBodyInterface_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp TorqueBalancingReferencesGenerator.h TorqueBalancingReferencesGenerator.cpp)

target_link_libraries(${PROJECT_NAME} ${yarpWholeBodyInterface_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_subdirectory(app)
