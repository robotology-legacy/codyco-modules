/**
 * Copyright (C) 2014 CoDyCo
 * @author: Jorhabib Eljaik
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

#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "wholeBodyNeckVelocityModule.h"

using namespace yarp::os;

int main(int argc, char **argv)
{
    ResourceFinder resourceFinder;
    
    resourceFinder.setVerbose(true);
    resourceFinder.setDefaultConfigFile("default.ini");
    resourceFinder.setDefaultContext("wholeBodyNeckVelocity");
    resourceFinder.configure(argc,argv);
    
    if (resourceFinder.check("help")) {
      std::cout<< "Expected parameters: " << std::endl;
      std::cout<< "\t --robot    : e.g. icub, icubGazeboSim" << std::endl;
      std::cout<< "\t --rate     : Period used by the module. Default is 10ms " << std::endl;
      std::cout<< "\t --local    : Prefix of the ports opened by this module. Default, name of the module " << std::endl;
      return 0;
    }
    
    Network yarp;
    double networkTimeout = 10.0;
    if (!Network::checkNetwork(networkTimeout)) {
      std::cerr << "YARP network not available" << std::endl;
      return -1;
    }
    
    WholeBodyNeckVelocityModule neckVelmodule;
    return neckVelmodule.runModule(resourceFinder);
}