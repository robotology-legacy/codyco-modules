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
 * motor drive gains. To "excite" the parameters the module motorFrictionExcitation can be used: it makes
 * the robot move in a way that is suitable for performing this kind of identification. Once the 
 * parameters have been identified they can be used by the module jointTorqueControl to implement a joint
 * torque control.
 *
 * \section sec_description Description
 * 
 * We assume that the relationship between the motor PWM \f$V_m\f$ and the joint torque \f$\tau\f$ is described
 * by the following equation:
 * The reference PWM signal \f$v(t)\f$ is a sinusoid with constant frequency \f$w\f$ and increasing amplitude:
 * \f[
 * V_m  = k_t \tau + (k_{vp} s(\dot{q}) + k_{vn} s(-\dot{q})) \dot{q} + (k_{cp} s(\dot{q}) + k_{cn} s(-\dot{q})) \text{sign}(k_s \dot{q}),
 * \f]
 * where \f$ k_t, k_{vp}, k_{vn}, k_{cp}, k_{cn} \f$ are the parameters to identify, \f$ \dot{q} \f$ is the joint velocity,
 * \f$ s(x) \f$ is the step function (1 for \f$ x>0 \f$, 0 otherwise) and \f$ \text{sign}(x) \f$ is the sign function (1 for
 * \f$ x>0 \f$, -1 for \f$ x<0 \f$, 0 for \f$ x=0 \f$).
 * 
 * \section sec_configuration_file Configuration File
 * 
 *
 * \section sec_rpc_commands RPC Commands
 *
 * The module opens a YARP rpc port with name "\<moduleName>\rpc". 
 * The rpc commands accepted by this module are:
 * - save:  save the current state of the identification on text file (the file name can be set through the rpc parameter filename)
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
	string              moduleName;     ///< name of the module instance
	string              robotName;      ///< name of the robot
    double              modulePeriod;   ///< module period in seconds
    int                 threadPeriod;   ///< thread period in milliseconds
    VectorXi            jointList;      ///< IDs of the joints that are specified in the configuration file
    vector<string>      jointNames;     ///< names of the joints that are specified in the configuration file
    double              avgTime, stdDev, avgTimeUsed, stdDevUsed;

	Port                rpcPort;		///< a port to handle rpc messages
    ParamHelperServer*  paramHelper;    ///< helper class for rpc set/get commands and streaming data
    wholeBodyInterface* robotInterface; ///< interface to communicate with the robot
    MotorFrictionIdentificationThread*   identificationThread;     ///< MotorFrictionIdentification control thread

public:
    MotorFrictionIdentificationModule();

	bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
	bool interruptModule();                       // interrupt, e.g., the ports 
	bool close();                                 // close and shut down the module
	bool respond(const Bottle& command, Bottle& reply);
	inline double getPeriod(){ return modulePeriod;  }
	bool updateModule();
    void commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply);

};

}

#endif
//empty line to make gcc happy

