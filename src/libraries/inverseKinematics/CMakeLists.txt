cmake_minimum_required(VERSION 2.8.12)

project(inverseKinematics CXX)

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

find_package(iDynTree REQUIRED)
find_package(IPOPT REQUIRED)
find_package(Eigen3 REQUIRED)


set(SOURCES src/InverseKinematics.cpp)
set(INCLUDES include/kinematics/InverseKinematics.h)

set(PRIVATE_SOURCES src/InverseKinematicsNLP.cpp
                    src/InverseKinematicsData.cpp
                    src/Transform.cpp)
set(PRIVATE_INCLUDES include/kinematics/private/InverseKinematicsNLP.h
                     include/kinematics/private/InverseKinematicsData.h
                     include/kinematics/private/Transform.h)

source_group("Private\\Header Files" FILES ${PRIVATE_INCLUDES})
source_group("Private\\Source Files" FILES ${PRIVATE_SOURCES})

add_library(${PROJECT_NAME} ${INCLUDES} ${SOURCES} 
                            ${PRIVATE_SOURCES} ${PRIVATE_INCLUDES})

include_directories(include/kinematics)
include_directories(include/kinematics/private)
include_directories(SYSTEM ${IPOPT_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${iDynTree_LIBRARIES} ${IPOPT_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# if(CODYCO_BUILD_TESTS)
    add_subdirectory(tests)
# endif()
