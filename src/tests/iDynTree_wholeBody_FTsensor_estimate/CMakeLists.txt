# Copyright: 2010-2011 RobotCub Consortium
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 


INCLUDE_DIRECTORIES(${iDynTree_INCLUDE_DIRS}
                    ${Orocos-KDL_INCLUDE_DIRS}
                    ${kdl_codyco_INCLUDE_DIRS}
                    ${paramHelp_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})
                    
ADD_EXECUTABLE(iDynTree_invDyn_test main.cpp)

TARGET_LINK_LIBRARIES(iDynTree_invDyn_test iDyn iDynTree ${kdl_codyco_LIBRARIES} ${YARP_LIBRARIES})

