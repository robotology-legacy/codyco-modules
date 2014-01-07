/*
 * Copyright (C) 2013 CODYCO Project
 * Author: Serena Ivaldi
 * email:  serena.ivaldi@isir.upmc.fr
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 @ingroup codyco_modules
 
 \defgroup reachComBalance reachComBalance
 
 A module for providing an interface for performing reaching actions on the robot while balancing. The movement control is based on ISIR controller.
 
 \section lib_sec Libraries
 - YARP libraries.
 - iCub libraries.
 - CODYCO libraries: modHelp
  
 \section example_sec Example
 
 In a terminal, simply launch
 
 \code
 reachComBalance --from default.ini
 \endcode
 
 then to send commands open a rpc port:
 
 \code
 yarp rpc --client /myPort
 \endcode
 
 and connect it to the command port
 
 \code
 yarp connect /myPort /reachComBalance/rpc:i
 \endcode
 
 finally type commands in the rpc port, like:
 
 \code
 calibration
 goto right_arm -0.3 0.1 0.05
 \endcode
 
 \section tested_os_sec Tested OS
 Windows
 
 \author Serena Ivaldi
 */

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <modHelp/modHelp.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "reachComBalance.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace modHelp;
using namespace std;

// necessary for cartesian interfaces
YARP_DECLARE_DEVICES(icubmod)


//---------------------------------------------------------
//                  MAIN
//---------------------------------------------------------

int main (int argc, char * argv[])
{
    YARP_REGISTER_DEVICES(icubmod)
    
    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP network not available. Aborting."<<endl;
        return -1;
    }
    
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("defaultSim.ini");         
    rf.setDefaultContext("reachComBalance/conf"); 
    rf.configure("ICUB_ROOT",argc,argv);
    
    if (rf.check("help"))
    {
        cout<< "Possible parameters"                                                                                                                                          << endl << endl;
        cout<< "\t--context          :Where to find an user defined .ini file "                                   <<endl;
        cout<< "\t--from             :Name of the file.ini to be used for calibration."                                                                                       <<endl;
        
        cout<<"The list of parameters is huge: please fill the configuration file.ini"<<endl;
        
        return 0;
    }
    
    //Creating the module
    ISIR_Balancer balancerModule;
    balancerModule.runModule(rf);
    
    return 0;
}
