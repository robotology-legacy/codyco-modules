cmake_minimum_required(VERSION 2.8.12)

project(TorqueBalancingReferenceGeneration CXX)

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

find_package(YARP REQUIRED)
find_package(iDynTree REQUIRED)
find_package(IPOPT REQUIRED)
find_package(Eigen3 REQUIRED)


set(SOURCES src/main.cpp src/OptimProblem.cpp src/Solver.cpp src/SolverData.cpp)
set(INCLUDES include/OptimProblem.h include/Solver.h include/SolverData.h)

add_executable(jointReferenceGenerator ${INCLUDES} ${SOURCES})

include_directories(include)
include_directories(SYSTEM ${IPOPT_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(jointReferenceGenerator ${YARP_LIBRARIES} ${iDynTree_LIBRARIES} ${IPOPT_LIBRARIES})
