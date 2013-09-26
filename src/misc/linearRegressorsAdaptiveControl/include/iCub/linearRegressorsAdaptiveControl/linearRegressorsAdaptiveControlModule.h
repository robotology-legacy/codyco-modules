
/*
 * Copyright (C) 2013 Italian Institute of Technology CoDyCo Project
 * Authors: Daniele Pucci and Silvio Traversaro
 * email:   daniele.pucci@iit.it and silvio.traversaro@iit.it
 * website: www.codyco.eu
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
*
@ingroup codyco_module
\defgroup icub_linearRegressorsAdaptiveControl linearRegressorsAdaptiveControl

This module is awesome.


\section intro_sec Description

\section lib_sec Libraries
YARP.


\section parameters_sec Parameters

<b>Command-line Parameters</b>

The following key-value pairs can be specified as command-line parameters by prefixing -- to the key
(e.g. --from file.ini. The value part can be changed to suit your needs; the default values are shown below.
 - \c from \c skinDriftCompensationLeft.ini \n
   specifies the configuration file
 - \c context \c graspingDemo/conf \n
    specifies the sub-path from \c $ICUB_ROOT/app to the configuration file
 - \c name \c skinDriftCompensation \n
    specifies the name of the module (used to form the stem of module port names)
 - \c robot \c icub \n
    specifies the name of the robot (used to form the root of robot port names)

<b>Configuration File Parameters </b>
 The following key-value pairs can be specified as parameters in the configuration file
 (they can also be specified as command-line parameters if you so wish).
 The value part can be changed to suit your needs; the default values are shown below.
 - \c hand \c right \n
   specifies which hand sensors has to be read
 - \c minBaseline \c 3 \n
   if the baseline of one sensor (at least) reaches this value, then the calibration is executed (if allowed)
 - \c zeroUpRawData \c false \n
   if true the raw data are considered from zero up, otherwise from 255 down
 - \c binarization \c false \n
   if true the output tactile data are binarized: 0 indicates no touch, whereas 100 indicates touch
 - \c smoothFilter \c false \n
   if true the output tactile data are filtered with an exponential moving average:
		y(t) = alpha*x(t) + (1-alpha)*y(t-1)
 - \c smoothFactor \c 0.5 \n
   alfa value of the smoothing filter



\section portsa_sec Ports Accessed

\section portsc_sec Ports Created
<b>Output ports </b>

<b>Input ports: </b>
All the port names listed below will be prefixed by \c /moduleName or whatever else is specified by the name parameter.\n
- /skinComp/right  or  /skinComp/left :
	port used by the IAnalogSensor interface for connecting with the sensors for reading the raw sensor data
	and sending calibration signals to the microprocessor
- /rpc:i: input ports to control the module, accepts a yarp::os::Bottle which contains one of these commands:
	- “forbid calibration”: prevent the module from executing the automatic sensor calibration
	- “allow calibration”: enable the automatic sensor calibration
	- “force calibration”: force the sensor calibration
	- "get percentile": return a yarp::os::Bottle containing the 95 percentile values of the tactile sensors
	- "set binarization": enable or disable the binarization (specifying the value on/off)
	- "set smooth filter": enable or disable the smooth filter (specifying the value on/off)
	- "set smooth factor": set the value of the smooth factor (in [0,1])
	- "help": get a list of the commands accepted by this module
	- "quit": quit the module


\section in_files_sec Input Data Files
None.


\section out_data_sec Output Data Files
None.


\section conf_file_sec Configuration Files
None.


\section tested_os_sec Tested OS Linux.


\section example_sec Example Instantiation of the Module
skinDriftCompensation --context graspingDemo/conf --from skinDriftCompensationRight.ini


\author Daniele Pucci and Silvio Traversaro

Copyright (C) 2013 CoDyCo Project

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at ICUB_HOME/main/src/modules/skinDriftCompensation/include/iCub/skinDriftCompensation/SkinDriftCompensation.h.
**/

#ifndef __ICUB_LINREGADAPTCTRL_H__
#define __ICUB_LINREGADAPTCTRL_H__

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>

#include <wbiy/wbiy.h>

#include <iCub/iDynTree/iCubTree.h>


#include "iCub/linearRegressorsAdaptiveControl/linearRegressorsAdaptiveControlThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

using namespace iCub::iDynTree;
using namespace wbiy;


namespace iCub{

namespace linearRegressorsAdaptiveControl{

class linearRegressorsAdaptiveControlModule:public RFModule
{
public:

	// the last element of the enum (COMMANDS_COUNT) represents the total number of commands accepted by this module
	typedef enum {
		help,				
        quit, 
        setGamma,
        setKappa,
        setLambda,
        setTrajTime,
		COMMANDS_COUNT} linearRegressorsAdaptiveControlCommand;

	bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
	bool interruptModule();                       // interrupt, e.g., the ports
	bool close();                                 // close and shut down the module
	bool respond(const Bottle& command, Bottle& reply);
	double getPeriod();
	bool updateModule();

private:
	// module default values
	static const int PERIOD_DEFAULT;
    static const int JOINT;
	static const string MODULE_NAME_DEFAULT;
	static const string ROBOT_NAME_DEFAULT;
	static const string RPC_PORT_DEFAULT;


	// module constants
	static const string COMMAND_LIST[];						// list of commands received through the rpc port
	static const string COMMAND_DESC[];						// descriptions of the commands

	/* module parameters */
	string moduleName;
	string robotName;

	// names of the ports
	string handlerPortName;

	/* class variables */
	Port handlerPort;									// a port to handle messages

	/* pointer to a new thread to be created and started in configure() and stopped in close() */
	linearRegressorsAdaptiveControlThread *controlThread;
    
    //Dynamical model passed to the thread
    iCubTree * icub_dynamical_model;
    
    icubWholeBodyInterface * icub_interface;
    
    //Actuated DOF
    std::vector<bool> controlled_DOFs;

	bool identifyCommand(Bottle commandBot, linearRegressorsAdaptiveControlCommand &com);

};

} //namespace iCub

} //namespace skinCalib

#endif // __ICUB_SKINCALIB_H__
//empty line to make gcc happy

