/*
* Copyright (C) 2014 ...
* Author: ...
* email: ...
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
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "module.h"


using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;



int main (int argc, char * argv[])
{
    //Creating and preparing the Resource Finder
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("randomJointsDefault.ini"); //default config file name.
    rf.setDefaultContext("reachRandomJointPositions"); //when no parameters are given to the module this is the default context
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout<< "Possible parameters" << endl << endl;
        cout<< "\t--context :Where to find an user defined .ini file e.g. /reachRandomJointPositions" <<endl;
        cout<< "\t--from :Name of the file.ini to be used for configuration." <<endl;
        cout<< "\t--rate :Period used by the module. Default set to 10ms." <<endl;
        cout<< "\t--robot :Robot name (icubSim or icub). Set to icub by default." <<endl;
        cout<< "\t--local :Prefix of the ports opened by the module. Set to the module name by default, i.e. reachRandomJointPositions." <<endl;
        cout<< "\t--boringMode :Reach all the points in the desired workspace with in a deterministic way"<<endl;
        return 0;
    }

    Network yarp;

    double network_timeout = 10.0;
    if (!yarp.checkNetwork(network_timeout))
    {
        fprintf(stderr,"Sorry YARP network is not available\n");
        return -1;
    }

    //Creating the module
    reachRandomJointPositionsModule module;
    return module.runModule(rf);
}


