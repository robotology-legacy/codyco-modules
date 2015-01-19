/*
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
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

#ifndef __LOCOMOTION_MODULE_H__
#define __LOCOMOTION_MODULE_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Vocab.h>

#include <paramHelp/paramHelperServer.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <locomotion/locomotionThread.h>

using namespace std;
using namespace yarp::os;
using namespace paramHelp;
using namespace wbi;

namespace locomotion
{

class LocomotionModule: public RFModule, public CommandObserver
{
    /* module parameters */
	string  moduleName;
	string  robotName;
    int     period;
    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;

	Port                rpcPort;		// a port to handle rpc messages
	LocomotionThread*   ctrlThread;     // locomotion control thread
    ParamHelperServer*  paramHelper;    // helper class for rpc set/get commands and streaming data
    wholeBodyInterface* robotInterface; // interface to communicate with the robot

public:
    LocomotionModule();

	bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
	bool interruptModule();                       // interrupt, e.g., the ports
	bool close();                                 // close and shut down the module
	bool respond(const Bottle& command, Bottle& reply);
	double getPeriod(){ return period;  }
	bool updateModule();

    void commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply);

};

}

#endif
//empty line to make gcc happy

