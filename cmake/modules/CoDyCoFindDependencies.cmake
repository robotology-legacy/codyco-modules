#### Find Eigen3
if (${CODYCO_USES_EIGEN_320})
	find_package(Eigen3 3.2 REQUIRED)
else()
	find_package(Eigen3 3.0 REQUIRED)
endif()

find_package(iDynTree REQUIRED)
find_package(paramHelp 0.0.3 REQUIRED)
