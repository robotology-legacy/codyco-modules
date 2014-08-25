/*
 * Copyright (C) 2014 CoDyCo
 * Author: Daniele Pucci, Francesco Romano
 * email:  daniele.pucci@iit.it, francesco.romano@iit.it
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

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <iostream>
#include "AdaptiveControlModule.h"

using namespace yarp::os;

int main(int argc, char **argv)
{
    ResourceFinder *rf = new ResourceFinder;
    
    rf->setVerbose(true);
    rf->setDefaultConfigFile("default.ini");         //default config file name.
    rf->setDefaultContext("adaptiveControl/conf"); //when no parameters are given to the module this is the default context
    rf->configure("ICUB_ROOT",argc,argv);
    
    if (rf->check("help"))
    {
        std::cout<< "Possible parameters" << std::endl << std::endl;
        std::cout<< "\t--context          :Where to find a user defined .ini file within $ICUB_ROOT/app e.g. /adaptiveControl/conf" << std::endl;
//        std::cout<< "\t--from             :Name of the file.ini to be used for calibration." << std::endl;
        std::cout<< "\t--rate             :Period used by the module. Default set to 10ms." << std::endl;
        std::cout<< "\t--robot            :Robot name (icubSim or icub). Set to icub by default." << std::endl;
        std::cout<< "\t--local            :Prefix of the ports opened by the module. Set to the module name by default, i.e. adaptiveControl." << std::endl;
        return 0;
    }
    
    Network yarp;
    
    if (!yarp.checkNetwork())
    {
        std::cerr << "Sorry YARP network is not available\n";
        return -1;
    }

    std::cout  << "Passive joint " << adaptiveControl::robotPartStartingIndex + adaptiveControl::passiveJointIndex << "    Active joint: " << adaptiveControl::robotPartStartingIndex + adaptiveControl::activeJointIndex << "\n";
    
    //Creating the module
    adaptiveControl::AdaptiveControlModule module;
    return module.runModule(*rf);
}
