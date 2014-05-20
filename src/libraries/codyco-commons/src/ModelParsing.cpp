/*
 * Copyright (C) 2014 RobotCub Consortium
 * Author: Francesco Romano
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "ModelParsing.h"
#include <iostream>
#include <yarp/os/ResourceFinder.h>
#include "iCub/iDynTree/iCubTree.h"

namespace codyco {

    bool iCubVersionFromRf(yarp::os::ResourceFinder & rf,
                           iCub::iDynTree::iCubTree_version_tag & icub_version)
    {
        //Checking iCub parts version
        /// \todo this part should be replaced by a more general way of accessing robot parameters
        ///       namely urdf for structure parameters and robotInterface xml (or runtime interface) to get available sensors
        icub_version.head_version = 2;
        if( rf.check("headV1") ) {
            icub_version.head_version = 1;
        }
        if( rf.check("headV2") ) {
            icub_version.head_version = 2;
        }

        icub_version.legs_version = 2;
        if( rf.check("legsV1") ) {
            icub_version.legs_version = 1;
        }
        if( rf.check("legsV2") ) {
            icub_version.legs_version = 2;
        }

        /// \note if feet_version are 2, the presence of FT sensors in the feet is assumed
        icub_version.feet_ft = true;
        if( rf.check("feetV1") ) {
            icub_version.feet_ft = false;
        }
        if( rf.check("feetV2") ) {
            icub_version.feet_ft = true;
        }

        #ifdef CODYCO_USES_URDFDOM
        if( rf.check("urdf") )
        {
            icub_version.uses_urdf = true;
            icub_version.urdf_file = rf.find("urdf").asString().c_str();
        }
        #endif
    }

    void iCubPartVersionOptionsPrint()
    {
        std::cout << "\t--headV1/headV2    :Version of the head."  <<std::endl;
        std::cout << "\t--legsV1/legsV2    :Version of the legs."  <<std::endl;
        std::cout << "\t--feetV1/feetV2    :Version of the feet."  <<std::endl;
        #ifdef CODYCO_USES_URDFDOM
        std::cout << "\t--urdf             :URDF file to load."    <<std::endl;
        #endif
    }
}
