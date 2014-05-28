# Copyright (C) 2010 RobotCub Consortium
# Author: Francesco Romano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


enable_testing()

include_directories(${codyco_commons_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR})

add_executable(codycoCommons-tests main.cpp)

target_link_libraries(codycoCommons-tests codyco_commons)

add_test(NAME test_codycoCommons-tests COMMAND codycoCommons-tests)
