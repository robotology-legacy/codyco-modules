SET(PROJECTNAME inertiaObserver)

PROJECT(${PROJECTNAME})

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${learningMachine_INCLUDE_DIRS}
                    ${iDyn_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${GSL_INCLUDE_DIRS})


ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iDyn
                                     learningMachine
                                     ${YARP_LIBRARIES}
                                     ${GSL_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
