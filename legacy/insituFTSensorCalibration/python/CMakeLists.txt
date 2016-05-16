# Copyright: (C) 2014-2015
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU LGPL v2.0+.

file(GLOB python_tools FTSensCalibManipulation.py)

yarp_install( PROGRAMS ${python_tools}
              DESTINATION bin)

