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

#ifndef LOCOMOTION_PLAN_CONSTANTS
#define LOCOMOTION_PLAN_CONSTANTS

#include <paramHelp/paramHelp.h>
#include <Eigen/Core>                               // import most common Eigen types
#include <vector>
#include <string>
#include <locomotion/locomotionConstants.h>

using namespace paramHelp;
using namespace Eigen;
using namespace std;

namespace locomotionPlanner
{

// *** DEFAULT PARAMETER VALUES
static const string         DEFAULT_MODULE_NAME     = "locomotionPlanner";  
static const int            DEFAULT_CTRL_PERIOD     = 10;                   // planner period in ms
static const string         DEFAULT_ROBOT_NAME      = "icubSim";            // robot name
// Streaming parameters
static const Vector2d       DEFAULT_XDES_COM        = Vector2d::Constant(0.0);
static const Vector7d       DEFAULT_XDES_FOOT       = Vector7d::Constant(0.0);
static const VectorNd       DEFAULT_QDES            = VectorNd::Constant(0.0);
static const Matrix4d       DEFAULT_H_W2B           = Matrix4d::Identity();


// *** IDs of all the module parameters
enum LocomotionPlannerParamId { 
    PARAM_ID_MODULE_NAME,       PARAM_ID_CTRL_PERIOD,       PARAM_ID_ROBOT_NAME,
    PARAM_ID_XDES_COM,          PARAM_ID_XDES_FOOT,         PARAM_ID_QDES,
    PARAM_ID_H_W2B,
    PARAM_ID_SIZE
};

// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
// ******************************************************************************************************************************
const ParamDescription locomotionPlannerParamDescr[]  = 
{ 
//               NAME               ID                          TYPE                SIZE                        BOUNDS                              I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
ParamDescription("name",            PARAM_ID_MODULE_NAME,       PARAM_DATA_STRING,  1,                          PARAM_BOUNDS_INF,                   PARAM_CONFIG,       &DEFAULT_MODULE_NAME,           "Name of the instance of the module"), 
ParamDescription("period",          PARAM_ID_CTRL_PERIOD,       PARAM_DATA_INT,     1,                          ParamBounds(1, 1000),               PARAM_CONFIG,       &DEFAULT_CTRL_PERIOD,           "Period of the control loop (ms)"), 
ParamDescription("robot",           PARAM_ID_ROBOT_NAME,        PARAM_DATA_STRING,  1,                          PARAM_BOUNDS_INF,                   PARAM_CONFIG,       &DEFAULT_ROBOT_NAME,            "Name of the robot"), 
// ************************************************* RPC PARAMETERS ****************************************************************************************************************************************************************************************************************************
// ************************************************* STREAMING INPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
// ************************************************* STREAMING OUTPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
};


// *** IDs of all the module command
enum LocomotionPlannerCommandId { 
    COMMAND_ID_START,   COMMAND_ID_STOP,    COMMAND_ID_HELP,    COMMAND_ID_QUIT, 
    COMMAND_ID_SIZE
};
// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE COMMANDS ********************************************
// ******************************************************************************************************************************
const CommandDescription locomotionPlannerCommandDescr[]  = 
{ 
//                  NAME            ID                          DESCRIPTION
CommandDescription("start",         COMMAND_ID_START,           "Start the planner"), 
CommandDescription("stop",          COMMAND_ID_STOP,            "Stop the planner"), 
CommandDescription("help",          COMMAND_ID_HELP,            "Get instructions about how to communicate with this module"), 
CommandDescription("quit",          COMMAND_ID_QUIT,            "Stop the planner and quit the module"), 
};

}

#endif
