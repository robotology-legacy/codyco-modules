/**
Copyright (C) 2013 CoDyCo
Author: Andrea Del Prete
CopyPolicy: Released under the terms of the GNU GPL v2.0.

\author Andrea Del Prete
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

#include "locomotion/locomotionConstants.h"
#include "locomotion/locomotionThread.h"
#include "locomotion/locomotionModule.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace paramHelp;
using namespace wbiy;
using namespace locomotion;

LocomotionModule::LocomotionModule()
{
    ctrlThread      = 0;
    robotInterface  = 0;
    paramHelper     = 0;
    period          = 10;
}
    
bool LocomotionModule::configure(ResourceFinder &rf)
{		
	string fwdSlash = "/";

    //-----------------GET THE MODULE NAME-------------------//
    string name = "locomotionControl";
    if (rf.check("name"))
        name = rf.find("name").asString().c_str();
    setName(name.c_str());
        
    //-----------------GET THE PERIOD-------------------//
    period = DEFAULT_CTRL_PERIOD;
    if (rf.check("period"))
        period = rf.find("period").asInt();
    
	//-----------------GET THE ROBOT NAME-------------------//
	string robot_name = DEFAULT_ROBOT_NAME;
	if (rf.check("robot"))
        robot_name = rf.find("robot").asString().c_str();

    //---------------------RPC PORT--------------------------//
    rpcPort.open(("/"+name+"/rpc").c_str());
    attach(rpcPort);

    //--------------------------WHOLE BODY INTERFACE--------------------------
    robotInterface = new icubWholeBodyInterface(name.c_str(), robot_name.c_str());
    robotInterface->addJoints(ICUB_MAIN_JOINTS);
    if(!robotInterface->init())
    {
        fprintf(stderr, "Error while initializing whole body interface. Closing module\n");
        return false;
    }

    //--------------------------PARAMETER HELPER--------------------------
    paramHelper = new ParamHelper();
    paramHelper->addParams(locomotionParamDescr, PARAM_ID_SIZE);

    //--------------------------THREAD--------------------------
    ctrlThread = new LocomotionThread(name, robot_name, (int)period);
    fprintf(stderr,"Locomotion control thread istantiated...\n");
    if(!ctrlThread->start())
    {
        fprintf(stderr, "Error while initializing locomotion control thread. Closing module.\n");
        return false;
    }
    fprintf(stderr,"Locomotion control started\n");
	
	return true;
}

bool LocomotionModule::respond(const Bottle& command, Bottle& reply) 
{
	if(paramHelper->respond(command, reply))
        return true;
    
    // here deal with any rpc command that is not a set/get
	reply.addString( (command.toString().c_str()+string(" command received.")).c_str());
	return true;	
}

bool LocomotionModule::interruptModule()
{
    if(ctrlThread)
        ctrlThread->suspend();
    rpcPort.interrupt();
    return true;
}

bool LocomotionModule::close()
{
	//stop threads
	if(ctrlThread)
    { 
        ctrlThread->stop(); 
        delete ctrlThread; 
        ctrlThread = 0; 
    }
		
	//closing ports
    rpcPort.interrupt();
	rpcPort.close();

    printf("[PERFORMANCE INFORMATION]:\n");
    printf("Expected period %3.1f ms.\nReal period: %3.1f+/-%3.1f ms.\n", period, avgTime, stdDev);
    printf("Real duration of 'run' method: %3.1f+/-%3.1f ms.\n", avgTimeUsed, stdDevUsed);
    if(avgTimeUsed<0.5*period)
        printf("Next time you could set a lower period to improve the controller performance.\n");
    else if(avgTime>1.3*period)
        printf("The period you set was impossible to attain. Next time you could set a higher period.\n");

    return true;
}

bool LocomotionModule::updateModule()
{
    if (ctrlThread==0)
    {
        printf("ControlThread pointers are zero\n");
        return false;
    }

    ctrlThread->getEstPeriod(avgTime, stdDev);
    ctrlThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()
    if(avgTime > 1.3 * period)
    {
        printf("[WARNING] Control loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %3.3f.\n", avgTime, stdDev, period);
        printf("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }

    return true;
}