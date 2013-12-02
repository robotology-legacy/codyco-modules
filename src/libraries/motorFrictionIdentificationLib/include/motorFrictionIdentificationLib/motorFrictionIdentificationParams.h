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

#ifndef __MOTOR_FRICTION_IDENTIFICATION_PARAMS
#define __MOTOR_FRICTION_IDENTIFICATION_PARAMS

#include <paramHelp/paramProxyBasic.h>
#include <Eigen/Core>                               // import most common Eigen types
#include <vector>
#include <string>
#include <yarp/os/Bottle.h>

using namespace paramHelp;
using namespace Eigen;
using namespace std;

namespace motorFrictionIdentificationLib
{
static const int       ICUB_DOFS = 25;    // number of (the main) degrees of freedom of iCub

// define some types
typedef yarp::sig::Matrix                          MatrixY;    // to not mistake Yarp Matrix and Eigen Matrix
typedef Eigen::Matrix<double,1,1>                  Vector1d;
typedef Eigen::Matrix<double,ICUB_DOFS,1>          VectorNd;
typedef Eigen::Matrix<int,ICUB_DOFS,1>             VectorNi;

// *** DEFAULT PARAMETER VALUES
static const string                 DEFAULT_MODULE_NAME     = "motorFrictionIdentificationControl"; ///< name of the module 
static const int                    DEFAULT_CTRL_PERIOD     = 10;                               ///< controller period in ms
static const string                 DEFAULT_ROBOT_NAME      = "icubSim";                        ///< robot name

// *** IDs of all the module command
enum MotorFrictionIdentificationCommandId { 
    COMMAND_ID_START,   COMMAND_ID_STOP,    COMMAND_ID_HELP,    COMMAND_ID_QUIT, 
    COMMAND_ID_SIZE
};

// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE COMMANDS ********************************************
// ******************************************************************************************************************************
const CommandDescription motorFrictionIdentificationCommandDescr[]  = 
{ 
//                  NAME            ID                          DESCRIPTION
CommandDescription("start",         COMMAND_ID_START,           "Start the controller"), 
CommandDescription("stop",          COMMAND_ID_STOP,            "Stop the controller"), 
CommandDescription("help",          COMMAND_ID_HELP,            "Get instructions about how to communicate with this module"), 
CommandDescription("quit",          COMMAND_ID_QUIT,            "Stop the controller and quit the module"), 
};


///< IDs of all the module parameters
enum MotorFrictionIdentificationParamId 
{ 
    PARAM_ID_MODULE_NAME,       PARAM_ID_CTRL_PERIOD,       PARAM_ID_ROBOT_NAME, 
    PARAM_ID_SIZE /*This is the number of parameters, so it must be the last value of the enum.*/
};

// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
// ******************************************************************************************************************************

const ParamProxyInterface *const motorFrictionIdentificationParamDescr[PARAM_ID_SIZE]  = 
{
// ************************************************* SIMPLE PARAMETERS ****************************************************************************************************************************************************************************************************************************************
//                          NAME                    ID                          SIZE                BOUNDS                                      I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
new ParamProxyBasic<string>("name",                 PARAM_ID_MODULE_NAME,       1,                                                              PARAM_CONFIG,       &DEFAULT_MODULE_NAME,           "Name of the instance of the module"), 
new ParamProxyBasic<int>(   "period",               PARAM_ID_CTRL_PERIOD,       1,                  ParamBilatBounds<int>(1,1000),              PARAM_CONFIG,       &DEFAULT_CTRL_PERIOD,           "Period of the control loop (ms)"), 
new ParamProxyBasic<string>("robot",                PARAM_ID_ROBOT_NAME,        1,                                                              PARAM_CONFIG,       &DEFAULT_ROBOT_NAME,            "Name of the robot"), 
// ************************************************* RPC PARAMETERS ****************************************************************************************************************************************************************************************************************************************
// ************************************************* STREAMING OUTPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
// ************************************************* STREAMING MONITOR PARAMETERS ****************************************************************************************************************************************************************************************************************************
};

}   // end namespace 

#endif
