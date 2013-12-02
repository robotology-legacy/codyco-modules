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
/**
 * @ingroup codyco_module
 * \defgroup motorFrictionIdentification motorFrictionIdentification
 *
 * The aim of this module is to identify the parameters characterizing the relationship between the motor 
 * PWM (proportional to voltage) and the joint torque. These parameters regards the joint friction and the 
 * motor drive gains.To "excite" the parameters the module motorFrictionExcitation can be used. 
 *
 * \section sec_description Description
 * 
 * 
 * 
 * \section sec_configuration_file Configuration File
 * 
 *
 * \section sec_rpc_commands RPC Commands
 *
 * The module opens a YARP rpc port with name "\<moduleName>\rpc". 
 * The rpc commands accepted by this module are:
 * - start: (re-)start the excitation
 * - stop:  stop the excitation
 * - help:  print a message describing all the rpc commands
 * - quit:  stop the excitation and quit the module
 *
 * \author Andrea Del Prete
 *
 * Copyright (C) 2013 CoDyCo Project
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at /src/modules/motorFrictionIdentification/include/motorFrictionIdentification/motorFrictionIdentificationModule.h .
 */

#ifndef __MOTOR_FRICTION_IDENTIFICATION_MODULE_H__
#define __MOTOR_FRICTION_IDENTIFICATION_MODULE_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Vocab.h>

#include <paramHelp/paramHelperServer.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <motorFrictionIdentification/motorFrictionIdentificationThread.h>
 
using namespace std;
using namespace yarp::os; 
using namespace paramHelp;
using namespace wbi;

namespace motorFrictionIdentification
{

class MotorFrictionIdentificationModule: public RFModule, public CommandObserver
{
    /* module parameters */
	string  moduleName;
	string  robotName;
    int     period;
    double  avgTime, stdDev, avgTimeUsed, stdDevUsed;

	Port                rpcPort;		// a port to handle rpc messages
	MotorFrictionIdentificationThread*   ctrlThread;     // MotorFrictionIdentification control thread
    ParamHelperServer*  paramHelper;    // helper class for rpc set/get commands and streaming data
    wholeBodyInterface* robotInterface; // interface to communicate with the robot

public:
    MotorFrictionIdentificationModule();

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

