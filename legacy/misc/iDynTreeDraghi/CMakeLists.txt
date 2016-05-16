
include_directories(${kdl_codyco_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS})

add_executable(iDynTreeDraghi main.cpp)

target_link_libraries(iDynTreeDraghi iDynTree ${orocos_kdl_LIBRARIES} ${kdl_codyco_LIBRARIES})