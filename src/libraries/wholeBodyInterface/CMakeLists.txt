# Copyright: (C) 2013 CoDyCo Consortium
# Authors: Andrea Del Prete
# CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version).

SET(PROJECTNAME wholeBodyInterface)

PROJECT(${PROJECTNAME})

SET(folder_source src/wbiUtil.cpp 
                  src/wbiConstants.cpp
                  src/iWholeBodyModel.cpp
                  src/iWholeBodyStates.cpp
                  src/iWholeBodySensors.cpp
                  src/iWholeBodyActuators.cpp
                  src/wholeBodyInterface.cpp)
                  
SET(folder_header include/wbi/wbi.h 
                  include/wbi/wbiConstants.h 
                  include/wbi/wbiUtil.h 
                  include/wbi/wbiUtil.inl
                  include/wbi/iWholeBodyModel.h
                  include/wbi/iWholeBodyStates.h
                  include/wbi/iWholeBodySensors.h
                  include/wbi/iWholeBodyActuators.h
                  include/wbi/wholeBodyInterface.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME})

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DESTINATION include/wbi
                                   FILES ${folder_header})


