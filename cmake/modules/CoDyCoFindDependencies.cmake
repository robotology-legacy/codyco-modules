#### Find Eigen3
find_package(Eigen3 3.2 REQUIRED)

find_package(orocos_kdl 1.2.3 QUIET)
if (NOT orocos_kdl_FOUND)
    set(OROCOS_KDL_OLDVERSION 1)
endif(NOT orocos_kdl_FOUND)

#### Find orocos_kdl, with fallback procedure to be compatible with ros installed version
find_package(orocos_kdl)
#support also for the old version of kdl cmake package
if(NOT orocos_kdl_FOUND)
   find_package(Orocos-KDL)
   if(NOT Orocos-KDL_FOUND)
      message(WARNING "KDL not found: neither orocos_kdl or Orocos-KDL cmake packages are available")
   else(NOT Orocos-KDL_FOUND)
      set(orocos_kdl_INCLUDE_DIRS ${Orocos-KDL_INCLUDE_DIRS})
      set(orocos_kdl_LIBRARY_DIRS ${Orocos-KDL_LIBRARY_DIRS})
      set(orocos_kdl_LIBRARIES ${Orocos-KDL_LIBRARIES})
      set(orocos_kdl_FOUND true)
   endif(NOT Orocos-KDL_FOUND)
endif(NOT orocos_kdl_FOUND)

#### Find kdl_codyco, require that kdl_codyco version
find_package(kdl_codyco 0.1.1 REQUIRED)


if(CODYCO_USES_URDFDOM)
    find_package(kdl_format_io)
    message(STATUS "kdl_format_io is version: ${kdl_format_io_VERSION}")
    if(NOT kdl_format_io_FOUND)
        message("Disabling URDF support as no kdl_format_io was found")
        set(CODYCO_USES_URDFDOM FALSE)
    endif()
endif()

find_package(iDynTree REQUIRED)
find_package(paramHelp REQUIRED)
