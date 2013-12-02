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
 * The aim of this module is to control the robot motors while the module motorFrictionIdentification
 * tries to identify the parameters characterizing the relationship between motor PWM and joint torque.
 * These parameters regards the joint friction and the motor drive gains.
 *
 * \section sec_description Description
 *
 * This module control the motor PWM of one motor at a time. The motors to control are specified in
 * the configuration file. In order not to excite the elasticity of the motor-link transmission
 * the PWM signals are low-frequency sinusoids. The amplitude of the sinusoids increases linearly with time
 * until a stop condition is met (e.g. a joint limit is too close).
 *
 * \section sec_excitation_phases Identification Phases
 *
 * To identify the parameters related to friction we need to move the joint at medium-high velocity.
 * On the other hand, to identify the motor drive gains we need to have medium-high joint torques.
 * If the load moved by the motor is light (e.g. elbow motor of a light humanoid) it is hard to get
 * high joint torques without external contacts. At the same time, it is hard to reach high joint velocities
 * in the presence of external contacts. For these reasons the module allows two types of excitations:
 * - free-motion excitation
 * - in-contact excitation
 *
 * \subsection ssec_free_motion_excitation Free-Motion Identification
 *
 * During this excitation phase the joint is supposed to move without any external contact. First, all the joints
 * of the robot are moved to a configuration specified for the calibration of the current joint to excite 
 * (see parameter initialJointConfiguration).
 * Typically, the joint to excite is moved to a position exactly in the middle between its upper and lower limits,
 * (which are specified in the configuration file). The PWM applied to the motor for keeping the joint in that position
 * is taken as offset \f$v_0\f$ for the PWM sinusoid, so that the joint will move around that position. At this point the motor
 * switches from position control to PWM control. The reference PWM signal \f$v(t)\f$ is a sinusoid with constant 
 * frequency \f$w\f$ and increasing amplitude:
 * \f[
 * v(t) = v_0 + (a t + a_0) \sin(w t),
 * \f]
 * where \f$a_0\f$ define the initial sinusoid amplitude, while \f$a\f$ defines its speed of increase.
 * The excitation phase stops when the joint gets too close to one its limits (see parameter jointLimitThreshold).
 * At the end of a free-motion excitation phase the module can either repeat the same excitation or proceed to the next 
 * one. The decision criteria is based on the covariance of the identified friction parameters. This information is
 * provided by the motorFrictionIdentification module. If the covariance is higher than the user-defined threshold then
 * the excitation process is repeated (see the parameter frictionParamCovarianceThreshold).
 *
 *
 * \subsection ssec_in_contact_excitation In-Contact Identification
 *
 * During this excitation phase the part of the robot that is moved by the joint to excite is supposed to be
 * in contact with the environment. To ease the contact phase, after reaching a user-defined joint configuration,
 * some joints (see parameter movableJoints) are set to "gravity compensation", so that the user can move them
 * to make the robot properly contact the environment. At this point the module waits for an input from the user 
 * before going on. After the user confirmation the module starts sending a sinusoidal reference PWM signal, 
 * of the same type of the free-motion excitation phase. The only difference with respect to the free-motion case
 * is that the stop condition takes into account also the contact force and moment. In particular, the control stops
 * when either the joint gets too close to one of its bounds (see the parameter jointLimitThreshold), or when the 
 * norm of the force/moment gets higher that a user-defined threshold (see the parameters contactForceThreshold and 
 * contactMomentThreshold).
 *
 * \section sec_configuration_file Configuration File
 *
 * Parameters for free-motion excitation:
 * - initial_joint_configuration
 * - a
 * - a0
 * - w
 * - joint_limit_thresh
 * - fric_param_covar_thresh
 *
 * Parameters for in-contact excitation:
 * - initial_joint_configuration
 * - a
 * - a0
 * - w
 * - movable_joints
 * - joint_limit_thresh
 * - contact_force_thresh
 * - contact_moment_thresh
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

