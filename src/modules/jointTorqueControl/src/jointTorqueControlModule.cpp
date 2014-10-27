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

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>

#include "jointTorqueControl/jointTorqueControlConstants.h"
#include "jointTorqueControl/jointTorqueControlThread.h"
#include "jointTorqueControl/jointTorqueControlModule.h"

using namespace yarp::dev;
using namespace paramHelp;
using namespace yarpWbi;
using namespace jointTorqueControl;

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

    //--------------------------WHOLE BODY INTERFACE--------------------------
    yarp::os::Property yarpWbiOptions;
    //Get wbi options from the canonical file
    if( !rf.check("wbi_conf_file") )
    {
        fprintf(stderr,"[ERR] jointTorqueControl: impossible to open wholeBodyInterface: wbi_conf_file option missing");
    }
    std::string wbiConfFile = rf.findFile("wbi_conf_file");
    yarpWbiOptions.fromConfigFile(wbiConfFile);

    robotInterface = new yarpWholeBodyInterface(moduleName.c_str(), yarpWbiOptions);

    wbiIdList RobotMainJoints;
    std::string RobotMainJointsListName = "ICUB_MAIN_JOINTS";
    if( !loadIdListFromConfig(RobotMainJointsListName,yarpWbiOptions,RobotMainJoints) )
    {
        fprintf(stderr, "[ERR] jointTorqueControl: impossible to load wbiId joint list with name %s\n",RobotMainJointsListName.c_str());
    }
    robotInterface->addJoints(RobotMainJoints);

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

double jointTorqueControlModule::getPeriod() { return 5; }

void jointTorqueControlModule::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case COMMAND_ID_HELP:
        paramHelper->getHelpMessage(reply);
        break;
    case COMMAND_ID_QUIT:
        stopModule();
        reply.addString("Quitting module!");
        break;
    }
}

bool jointTorqueControlModule::interruptModule(){ return true; }

bool jointTorqueControlModule::close()
{
	//stop threads
    if(ctrlThread){     ctrlThread->stop();         delete ctrlThread;      ctrlThread = 0;     }
    if(paramHelper){    paramHelper->close();       delete paramHelper;     paramHelper = 0;    }
    if(robotInterface){ robotInterface->close();    delete robotInterface;  robotInterface = 0; }

	//closing ports
    rpcPort.interrupt();
	rpcPort.close();
    cout << "about to close"<<endl;
    return true;
}

bool jointTorqueControlModule::updateModule()
{
     if (!ctrlThread) {
    printf("jointTorqueControlModule: Error, thread pointer is zero!\n");
    return false;
     }

    double periodMean = 0, periodStdDeviation = 0;
    double usedMean = 0, usedStdDeviation = 0;

    ctrlThread->getEstPeriod(periodMean, periodStdDeviation);
    ctrlThread->getEstUsed(usedMean, usedStdDeviation);

//     if(periodMean > 1.3 * period)
    {
        fprintf(stderr, "Control loop period: %3.3f+/-%3.3f. Expected period %d.\n", periodMean, periodStdDeviation, period);
        fprintf(stderr, "Duration of 'run' method: %3.3f+/-%3.3f.\n", usedMean, usedStdDeviation);
    }
    return true;
}
