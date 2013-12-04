/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Daniele Pucci
 * email:  daniele.pucci@iit.it
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

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>

#include "jointTorqueControl/jointTorqueControlConstants.h"
#include "jointTorqueControl/jointTorqueControlThread.h"
#include "jointTorqueControl/jointTorqueControlModule.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace paramHelp;
using namespace wbiIcub;
using namespace jointTorqueControl;
using namespace motorFrictionIdentificationLib;

jointTorqueControlModule::jointTorqueControlModule()
{
    ctrlThread      = 0;
    robotInterface  = 0;
    paramHelper     = 0;
    torqueCtrl      = 0;
    period          = 10;
}
    
bool jointTorqueControlModule::configure(ResourceFinder &rf)
{		
    //-------------------------------------------------- PARAMETER HELPER SERVER ---------------------------------------------------------
    paramHelper = new ParamHelperServer(jointTorqueControlParamDescr, PARAM_ID_SIZE, jointTorqueControlCommandDescr, COMMAND_ID_SIZE);
    paramHelper->linkParam(PARAM_ID_MODULE_NAME,	&moduleName);
    paramHelper->linkParam(PARAM_ID_CTRL_PERIOD,	&period);
    paramHelper->linkParam(PARAM_ID_ROBOT_NAME,		&robotName);

    paramHelper->registerCommandCallback(COMMAND_ID_HELP, this);
    paramHelper->registerCommandCallback(COMMAND_ID_QUIT, this);
	
    // Read parameters from configuration file (or command line)
    Bottle initMsg;
    paramHelper->initializeParams(rf, initMsg);
    printf("*** Parsing configuration file...\n%s\n", initMsg.toString().c_str());

    // Open ports for communicating with other modules
    if(!paramHelper->init(moduleName))
    { fprintf(stderr, "Error while initializing parameter helper. Closing module.\n"); return false; }
    rpcPort.open(("/"+moduleName+"/rpc").c_str());
    setName(moduleName.c_str());
    attach(rpcPort);

    initMsg.clear();
    
    //--------------------------WHOLE BODY INTERFACE--------------------------
    robotInterface = new icubWholeBodyInterface(moduleName.c_str(), robotName.c_str());
    robotInterface->addJoints(ICUB_MAIN_JOINTS);
    if(!robotInterface->init()){ fprintf(stderr, "Error while initializing whole body interface. Closing module\n"); return false; }

    //--------------------------CTRL THREAD--------------------------
    ctrlThread = new jointTorqueControlThread(period, moduleName, robotName, paramHelper, robotInterface);
    if(!ctrlThread->start()){ fprintf(stderr, "Error while initializing control torque thread. Closing module.\n"); return false; }
    
    fprintf(stderr,"Control torque module started\n");
	return true;
}

bool jointTorqueControlModule::respond(const Bottle& cmd, Bottle& reply) 
{
    paramHelper->lock();
	if(!paramHelper->processRpcCommand(cmd, reply)) 
	    reply.addString( (string("Command ")+cmd.toString().c_str()+" not recognized.").c_str());
    paramHelper->unlock();

    // if reply is empty put something into it, otherwise the rpc communication gets stuck
    if(reply.size()==0)
        reply.addString( (string("Command ")+cmd.toString().c_str()+" received.").c_str());
	return true;	
}

void jointTorqueControlModule::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case COMMAND_ID_HELP:   
        paramHelper->getHelpMessage(reply);     
        break;
    case COMMAND_ID_QUIT:   
        stopModule(); 
        reply.addString("Quitting module.");    
        break;
    }
}

bool jointTorqueControlModule::interruptModule()
{
    if(ctrlThread)  ctrlThread->stop();
    rpcPort.interrupt();
    return true;
}

bool jointTorqueControlModule::close()
{
	//stop threads
    if(ctrlThread){     ctrlThread->stop();         delete ctrlThread;      ctrlThread = 0;     }
    if(paramHelper){    paramHelper->close();       delete paramHelper;     paramHelper = 0;    }
    if(robotInterface){ robotInterface->close();    delete robotInterface;  robotInterface = 0; }

	//closing ports
	rpcPort.close();
    cout << "about to close"<<endl;
    return true;
}

bool jointTorqueControlModule::updateModule()
{
    if (ctrlThread!=0)
        return true;
    printf("jointTorqueControlModule: Error, thread pointer is zero!\n");
    return false;
}
