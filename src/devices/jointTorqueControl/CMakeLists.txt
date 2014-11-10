# Copyright: (C) 2014 Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

MACRO(codyco_export_library target)
  PARSE_ARGUMENTS(${target}
    "INTERNAL_INCLUDE_DIRS;EXTERNAL_INCLUDE_DIRS;DEPENDS;DESTINATION;FILES;FILES_WITH_PATH"
    "VERBOSE"
    ${ARGN}
    )

  set(VERBOSE ${${target}_VERBOSE})
  if(VERBOSE)
    MESSAGE(STATUS "*** Arguments for ${target}")
    MESSAGE(STATUS "Internal directories: ${${target}_INTERNAL_INCLUDE_DIRS}")
    MESSAGE(STATUS "External directories: ${${target}_EXTERNAL_INCLUDE_DIRS}")
    MESSAGE(STATUS "Dependencies: ${${target}_DEPENDS}")
    MESSAGE(STATUS "Destination: ${${target}_DESTINATION}")
    MESSAGE(STATUS "Header files: ${${target}_FILES}")
    MESSAGE(STATUS "Header files for which we keep the relative path: ${${target}_FILES_WITH_PATH}")
    MESSAGE(STATUS "Option verbosity: ${${target}_VERBOSE}")
  endif()

  set(internal_includes ${${target}_INTERNAL_INCLUDE_DIRS})
  set(external_includes ${${target}_EXTERNAL_INCLUDE_DIRS})
  set(dependencies ${${target}_DEPENDS})
  set(files ${${target}_FILES})
  set(files_with_path ${${target}_FILES_WITH_PATH})
  set(destination ${${target}_DESTINATION})

  set(ICUB_EXPORTBUILD_FILE icub-export-build.cmake)

  ##### Append target to global list.
  icub_set_property(GLOBAL APPEND PROPERTY ICUB_TARGETS ${target})
  # Install/export rules
  install(TARGETS ${target} EXPORT icub-targets LIBRARY DESTINATION lib ARCHIVE DESTINATION lib COMPONENT Development)
  if (MSVC)
    install ( FILES ${CMAKE_BINARY_DIR}/lib/Debug/${target}d.pdb
              DESTINATION lib
              CONFIGURATIONS Debug
              COMPONENT Development
              OPTIONAL)
  endif()
  export(TARGETS ${target} APPEND FILE ${CMAKE_BINARY_DIR}/${ICUB_EXPORTBUILD_FILE})
    #important wrap ${dependencies} with "" to allows storing a list of dependencies
  set_target_properties(${target} PROPERTIES DEPENDS "${dependencies}")

  ##### Handle include directories
  # Parsing dependencies
  if (dependencies)
    foreach (d ${dependencies})
        get_target_property(in_dirs ${d} INTERNAL_INCLUDE_DIRS)
        get_target_property(ext_dirs ${d} EXTERNAL_INCLUDE_DIRS)

        if (VERBOSE)
            message(STATUS "Getting from target ${d}:")
            message(STATUS "${in_dirs}")
            message(STATUS "${ext_dirs}")
        endif()

        if (in_dirs)
            set(internal_includes ${internal_includes} ${in_dirs})
        endif (in_dirs)

        if (ext_dirs)
            set(external_includes ${external_includes} ${ext_dirs})
        endif(ext_dirs)
    endforeach(d)
  endif(dependencies)
  ############################

  ################ Build unique variable with internal and external include directories
  ## Set corresponding target's properties
  set(include_dirs "")

  if (internal_includes)
    list(REMOVE_DUPLICATES internal_includes)
    set_target_properties(${target} PROPERTIES
                        INTERNAL_INCLUDE_DIRS
                        "${internal_includes}")
    if(VERBOSE)
        message(STATUS "Target ${target} exporting internal headers: ${internal_includes}")
    endif()
    set(include_dirs ${include_dirs} ${internal_includes})
  endif()

  if (external_includes)
    list(REMOVE_DUPLICATES external_includes)
    set_target_properties(${target} PROPERTIES
                        EXTERNAL_INCLUDE_DIRS
                        "${external_includes}")
    if(VERBOSE)
        message(STATUS "Target ${target} exporting external headers: ${external_includes}")
    endif()
    set(include_dirs ${include_dirs} ${external_includes})
  endif()

  if (include_dirs)
    list(REMOVE_DUPLICATES include_dirs)
    set_property(TARGET ${target} PROPERTY INCLUDE_DIRS  "${include_dirs}")
    if (VERBOSE)
        message(STATUS "Target ${target} exporting: ${include_dirs}")
    endif()
    set(${target}_INCLUDE_DIRS "${include_dirs}" CACHE STRING "include directories target" FORCE)
  endif()
  ##############################################

  # Compile libraries using -fPIC to produce position independent code
  # For CMAKE_VERSION >= 2.8.10 this is handled in iCubOptions.cmake
  # using the CMAKE_POSITION_INDEPENDENT_CODE flag
  if(CMAKE_COMPILER_IS_GNUCXX AND NOT BUILD_SHARED_LIBS)
    if(CMAKE_VERSION VERSION_EQUAL "2.8.9")
      set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
    elseif(CMAKE_VERSION VERSION_LESS "2.8.9")
      set_target_properties(${target} PROPERTIES COMPILE_FLAGS -fPIC)
    endif()
  endif()

  #### Export rules
  if (files AND destination)
    if (VERBOSE)
        message(STATUS "Target ${target} installing ${files} to ${destination}")
    endif()
    install(FILES ${files} DESTINATION ${destination} COMPONENT Development)

    set_target_properties(${target} PROPERTIES
                        HEADERFILES
                        "${files}")

    set_target_properties(${target} PROPERTIES
                        HEADERS_DESTINATION
                        ${destination})
  endif()

  #### Export rules for files_with_path case
  if (files_with_path AND destination)
    if (VERBOSE)
        message(STATUS "Target ${target} installing ${files_with_path} to ${destination}")
    endif()


    foreach(cur_file  ${files_with_path})
        get_filename_component(file_rel_dir ${cur_file} PATH)
        install(FILES ${cur_file} DESTINATION ${destination}/${file_rel_dir} COMPONENT Development)
    endforeach()
    set_target_properties(${target} PROPERTIES
                        HEADERFILES
                        "${files_with_path}")

    set_target_properties(${target} PROPERTIES
                        HEADERS_DESTINATION
                        ${destination})
  endif()

ENDMACRO(codyco_export_library)

PREPARE_DEVICE(passThroughControlBoard TYPE PassThroughControlBoard INCLUDE PassThroughControlBoard.h WRAPPER controlboardwrapper)
PREPARE_DEVICE(jointTorqueControl TYPE JointTorqueControl INCLUDE JointTorqueControl.h WRAPPER controlboardwrapper)

if(NOT SKIP_jointTorqueControl)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS})

yarp_add_plugin(passThroughControlBoard PassThroughControlBoard.h PassThroughControlBoard.cpp)
target_link_libraries(passThroughControlBoard ${YARP_LIBRARIES})

codyco_export_library(passThroughControlBoard)

yarp_add_plugin(jointTorqueControl JointTorqueControl.h JointTorqueControl.cpp PassThroughControlBoard.h  PassThroughControlBoard.cpp)
target_link_libraries(jointTorqueControl ${YARP_LIBRARIES})

codyco_export_library(jointTorqueControl)

endif (NOT SKIP_jointTorqueControl)




