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

#include <paramHelp/paramProxyBasic.h>
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
static const string         DEFAULT_MODULE_NAME             = "locomotionPlanner";
static const string         DEFAULT_ROBOT_NAME              = "icub";               // robot name
static const string         DEFAULT_LOCOMOTION_CTRL_NAME    = "locomotionControl";  // locomotion controller name
static const string         DEFAULT_FILE_NAME               = "ts20_invkin/iCubGenova01_100poses_A.txt";         // input params file (trajectory points)
// Streaming parameters


// *** IDs of all the module parameters
enum LocomotionPlannerParamId { 
    PARAM_ID_PLANNER_NAME,       PARAM_ID_ROBOT_NAME,   PARAM_ID_LOCOMOTION_CTRL_NAME,
    PARAM_ID_FILE_NAME,          PARAM_ID_SIZE
};

// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
// ******************************************************************************************************************************
const ParamProxyInterface *const locomotionPlannerParamDescr[]  = 
{ 
//                          NAME                   ID                              SIZE        BOUNDS                       I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
new ParamProxyBasic<string>("name",                PARAM_ID_PLANNER_NAME,          1,          ParamConstraint<string>(),   PARAM_CONFIG,       &DEFAULT_MODULE_NAME,           "Name of the instance of the module"), 
new ParamProxyBasic<string>("robot",               PARAM_ID_ROBOT_NAME,            1,          ParamConstraint<string>(),   PARAM_CONFIG,       &DEFAULT_ROBOT_NAME,            "Name of the robot"), 
new ParamProxyBasic<string>("locomotionCtrlName",  PARAM_ID_LOCOMOTION_CTRL_NAME,  1,          ParamConstraint<string>(),   PARAM_CONFIG,       &DEFAULT_LOCOMOTION_CTRL_NAME,  "Name of the locomotion controller module"), 
new ParamProxyBasic<string>("filename",            PARAM_ID_FILE_NAME,             1,          ParamConstraint<string>(),   PARAM_CONFIG,       &DEFAULT_FILE_NAME,             "Name of the file containing the via points"),
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
