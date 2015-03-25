# Copyright: (C) 2015 Robotics Brain and Cognitive Sciences, Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

SET(PROJECTNAME eventRepeater)
PROJECT(${PROJECTNAME})
include_directories(${YARP_INCLUDE_DIRS} ${generated_libs_include_dir})

ADD_EXECUTABLE (${PROJECTNAME} eventRepeaterModule.cpp)
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} eventRepeaterIDL)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)