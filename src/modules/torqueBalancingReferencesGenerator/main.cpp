/**
 * Copyright (C) 2015 CoDyCo
 * @author: Daniele Pucci
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

#include "TorqueBalancingReferencesGenerator.h"
#include <yarp/os/Network.h>

int main(int argc, char **argv)
{
    using namespace yarp::os;
    using namespace yarp::sig;

    if (!yarp::os::Network::checkNetwork(5.0)) {
        std::cout << "Yarp network not found\n";
        return false;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("torqueBalancingRefGen.ini");         //default config file name.
    rf.setDefaultContext("torqueBalancingReferencesGenerator"); //when no parameters are given to the module this is the default context
    rf.configure(argc, argv);
    TorqueBalancingReferencesGenerator mainModule;
    return mainModule.runModule(rf);
}
