/**
 * Copyright (C) 2014 CoDyCo
 * @author: Francesco Romano
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

#include "TorqueBalancingModule.h"
#include "TorqueBalancingController.h"

namespace codyco {
    namespace torquebalancing {
        
        TorqueBalancingModule::TorqueBalancingModule() {}
        TorqueBalancingModule::~TorqueBalancingModule() {}
        
        bool TorqueBalancingModule::configure(yarp::os::ResourceFinder& rf)
        {
            
            TorqueBalancingController *controller = new TorqueBalancingController(10);
            
            return true;
        }
        bool TorqueBalancingModule::updateModule() { return true; }
        bool TorqueBalancingModule::close() { return true; }
    }
}