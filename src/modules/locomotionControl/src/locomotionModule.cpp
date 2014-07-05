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
using namespace wbiIcub;
using namespace locomotion;

void iCubVersionFromRf(ResourceFinder & rf, iCub::iDynTree::iCubTree_version_tag & icub_version)
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

LocomotionModule::LocomotionModule()
{
    ctrlThread      = 0;
    robotInterface  = 0;
    paramHelper     = 0;
    period          = 10;
}

bool LocomotionModule::configure(ResourceFinder &rf)
{
    //--------------------------PARAMETER HELPER--------------------------
    paramHelper = new ParamHelperServer(locomotionParamDescr, PARAM_ID_SIZE, locomotionCommandDescr, COMMAND_ID_SIZE);
    paramHelper->linkParam(PARAM_ID_MODULE_NAME, &moduleName);
    paramHelper->linkParam(PARAM_ID_CTRL_PERIOD, &period);
    paramHelper->linkParam(PARAM_ID_ROBOT_NAME, &robotName);
    paramHelper->registerCommandCallback(COMMAND_ID_HELP, this);
    paramHelper->registerCommandCallback(COMMAND_ID_QUIT, this);

    // Read parameters from configuration file (or command line)
    Bottle initMsg;
    paramHelper->initializeParams(rf, initMsg);
    printBottle(initMsg);

    // Open ports for communicating with other modules
    if(!paramHelper->init(moduleName)){ fprintf(stderr, "Error while initializing parameter helper. Closing module.\n"); return false; }
    rpcPort.open(("/"+moduleName+"/rpc").c_str());
    setName(moduleName.c_str());
    attach(rpcPort);

    //--------------------------WHOLE BODY INTERFACE--------------------------
    iCub::iDynTree::iCubTree_version_tag icub_version;
    iCubVersionFromRf(rf,icub_version);
    robotInterface = new icubWholeBodyInterface(moduleName.c_str(), robotName.c_str(),icub_version);
    robotInterface->addJoints(ICUB_MAIN_JOINTS);
//    robotInterface->addEstimate(ESTIMATE_FORCE_TORQUE_SENSOR, LocalId(RIGHT_LEG,1));  // right ankle ft sens
//    robotInterface->addEstimate(ESTIMATE_FORCE_TORQUE_SENSOR, LocalId(LEFT_LEG,1));   // left ankle ft sens
    if(!robotInterface->init()){ fprintf(stderr, "Error while initializing whole body interface. Closing module\n"); return false; }

    //--------------------------CTRL THREAD--------------------------
    ctrlThread = new LocomotionThread(moduleName, robotName, period, paramHelper, robotInterface);
    if(!ctrlThread->start()){ fprintf(stderr, "Error while initializing locomotion control thread. Closing module.\n"); return false; }

    fprintf(stderr,"Locomotion control started\n");

    //-------------------------- CHECK startNow FLAG ---------------------
    if(rf.check("startNow"))
    {
        printf("startNow flag found => Gonna start the controller right away!\n");
        ctrlThread->startController();
    }

	return true;
}

bool LocomotionModule::respond(const Bottle& cmd, Bottle& reply)
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

void LocomotionModule::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
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
    if(ctrlThread){     ctrlThread->stop();         delete ctrlThread;      ctrlThread = 0;     }
    if(paramHelper){    paramHelper->close();       delete paramHelper;     paramHelper = 0;    }
    if(robotInterface)
    {
        bool res=robotInterface->close();
        if(res)
            printf("Error while closing robot interface\n");
        delete robotInterface;
        robotInterface = 0;
    }

	//closing ports
	rpcPort.close();

    printf("[PERFORMANCE INFORMATION]:\n");
    printf("Expected period %d ms.\nReal period: %3.1f+/-%3.1f ms.\n", period, avgTime, stdDev);
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
//#ifndef NDEBUG
    if(avgTime > 1.3 * period)
    {
        printf("[WARNING] Control loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %d.\n", avgTime, stdDev, period);
        printf("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
    }
//#endif

    return true;
}
