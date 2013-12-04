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
static const int            PARAM_NUMBER        = 5;    ///< number of estimated parameters

// define some types
typedef Eigen::Matrix<double,1,1>                   Vector1d;
typedef Eigen::Matrix<double,PARAM_NUMBER,1>        VectorPd;

///< Constants


// *** DEFAULT PARAMETER VALUES
static const string         DEFAULT_MODULE_NAME         = "motorFrictionIdentification";        ///< name of the module 
static const int            DEFAULT_CTRL_PERIOD         = 10;                                   ///< controller period in ms
static const string         DEFAULT_ROBOT_NAME          = "icubSim";                            ///< robot name
static const string         DEFAULT_OUTPUT_FILENAME     = "motorFrictionIdentification.ini";    ///< output configuration file name
static const double         DEFAULT_IDENTIF_DELAY       = 1.0;      ///< delay (in sec) used for the identification
static const double         DEFAULT_ZERO_VEL_THRESH     = 1.0;      ///< threshold to consider a velocity zero (deg/s)
static const int            DEFAULT_VEL_EST_WIND_SIZE   = 61;       ///< max number of samples used for velocity estimation
static const double         DEFAULT_FORGET_FACTOR       = 1.0;      ///< forgetting factor of the recursive least-square (in [0;1], where 1 means do not forget anything)
static const double         DEFAULT_COVARIANCE_INV      = 0.0;      ///< parameter identification covariance inverse used to start up the algorithm

///< IDs of all the module parameters
enum MotorFrictionIdentificationParamId 
{ 
    /* Input parameters */
    PARAM_ID_MODULE_NAME,       PARAM_ID_CTRL_PERIOD,       PARAM_ID_ROBOT_NAME, 
    PARAM_ID_OUTPUT_FILENAME,   PARAM_ID_JOINT_LIST,        PARAM_ID_ACTIVE_JOINTS,
    PARAM_ID_IDENTIF_DELAY,     PARAM_ID_ZERO_VEL_THRESH,   PARAM_ID_VEL_EST_WIND_SIZE,
    PARAM_ID_FORGET_FACTOR,     PARAM_ID_JOINT_TO_MONITOR,
    /* Input\output parameters */
    PARAM_ID_COVARIANCE_INV,    PARAM_ID_RHS,
    /* Monitor parameters */
    PARAM_ID_JOINT_VEL,         PARAM_ID_JOINT_TORQUE,      PARAM_ID_JOINT_VEL_SIGN,
    PARAM_ID_MOTOR_PWM,         PARAM_ID_PARAM_ESTIMATES,   PARAM_ID_PARAM_VARIANCE,
    PARAM_ID_SIZE /*This is the number of parameters, so it must be the last value of the enum.*/
};

// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
// ******************************************************************************************************************************

const ParamProxyInterface *const motorFrictionIdentificationParamDescr[PARAM_ID_SIZE]  = 
{
// ************************************************* SIMPLE PARAMETERS ****************************************************************************************************************************************************************************************************************************************
//                          NAME                    ID                          SIZE                BOUNDS                                  I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
new ParamProxyBasic<string>("name",                 PARAM_ID_MODULE_NAME,       1,                                                          PARAM_CONFIG,       &DEFAULT_MODULE_NAME,           "Name of the instance of the module"), 
new ParamProxyBasic<int>(   "period",               PARAM_ID_CTRL_PERIOD,       1,                  ParamBilatBounds<int>(1,1000),          PARAM_CONFIG,       &DEFAULT_CTRL_PERIOD,           "Period of the control loop (ms)"), 
new ParamProxyBasic<string>("robot",                PARAM_ID_ROBOT_NAME,        1,                                                          PARAM_CONFIG,       &DEFAULT_ROBOT_NAME,            "Name of the robot"), 
new ParamProxyBasic<int>(   "joint list",           PARAM_ID_JOINT_LIST,        PARAM_SIZE_FREE,    ParamBilatBounds<int>(0,100),           PARAM_CONFIG,       0,                              "List of the ids of the robot joints to use"), 
// ************************************************* RPC PARAMETERS ****************************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<string>("filename",             PARAM_ID_OUTPUT_FILENAME,   1,                                                          PARAM_IN_OUT,       &DEFAULT_OUTPUT_FILENAME,       "Name of the file on which to save the state of the identification"), 
new ParamProxyBasic<int>(   "active joints",        PARAM_ID_ACTIVE_JOINTS,     PARAM_SIZE_FREE,    ParamBilatBounds<int>(0,1),             PARAM_IN_OUT,       0,                              "List of flags (0,1) indicating for which motors the identification is active"), 
new ParamProxyBasic<double>("delay",                PARAM_ID_IDENTIF_DELAY,     1,                  ParamBilatBounds<double>(0.,10.),       PARAM_IN_OUT,       &DEFAULT_IDENTIF_DELAY,         "Delay (in sec) used before processing a sample to update the identified parameters"), 
new ParamProxyBasic<double>("zero vel thr",         PARAM_ID_ZERO_VEL_THRESH,   1,                  ParamBilatBounds<double>(0.,10.),       PARAM_IN_OUT,       &DEFAULT_ZERO_VEL_THRESH,       "Velocities (deg/sec) below this threshold are considered zero"), 
new ParamProxyBasic<int>(   "vel est wind",         PARAM_ID_VEL_EST_WIND_SIZE, 1,                  ParamBilatBounds<int>(1,1000),          PARAM_IN_OUT,       &DEFAULT_VEL_EST_WIND_SIZE,     "Max size of the moving window used for estimating joint velocities"), 
new ParamProxyBasic<double>("forget factor",        PARAM_ID_FORGET_FACTOR,     1,                  ParamBilatBounds<double>(0.,1.),        PARAM_IN_OUT,       &DEFAULT_FORGET_FACTOR,         "Forgetting factor (in [0,1], 1=do not forget) used in the identification"),
new ParamProxyBasic<int>(   "joint monitor",        PARAM_ID_JOINT_TO_MONITOR,  1,                  ParamBilatBounds<int>(0,1000),          PARAM_IN_OUT,       0,                              "Joint to monitor"),
// ************************************************* FILE OUTPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("covariance inv",       PARAM_ID_COVARIANCE_INV,    PARAM_SIZE_FREE,                                            PARAM_IN_OUT,       0,                              "Inverse of the covariance matrix of the parameter estimations"),
new ParamProxyBasic<double>("rhs",                  PARAM_ID_RHS,               PARAM_SIZE_FREE,                                            PARAM_IN_OUT,       0,                              "Right-hand side of the linear vector equation that is solved for estimating the parameters"),
// ************************************************* STREAMING MONITOR PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("dq",                   PARAM_ID_JOINT_VEL,         1,                                                          PARAM_MONITOR,      0,                              "Velocity of the monitored joint"),
new ParamProxyBasic<double>("torque",               PARAM_ID_JOINT_TORQUE,      1,                                                          PARAM_MONITOR,      0,                              "Torque of the monitored joint"),
new ParamProxyBasic<double>("sign dq",              PARAM_ID_JOINT_VEL_SIGN,    1,                                                          PARAM_MONITOR,      0,                              "Velocity sign of the monitored joint"),
new ParamProxyBasic<double>("pwm",                  PARAM_ID_MOTOR_PWM,         1,                                                          PARAM_MONITOR,      0,                              "Motor pwm of the monitored joint"),
new ParamProxyBasic<double>("estimates",            PARAM_ID_PARAM_ESTIMATES,   PARAM_NUMBER,                                               PARAM_MONITOR,      0,                              "Estimates of the parameters of the monitored joint"),
new ParamProxyBasic<double>("variances",            PARAM_ID_PARAM_VARIANCE,    PARAM_NUMBER,                                               PARAM_MONITOR,      0,                              "Variances of the parameters of the monitored joint")
};


///< IDs of all the module command
enum MotorFrictionIdentificationCommandId 
{ 
    COMMAND_ID_SAVE,    COMMAND_ID_HELP,    COMMAND_ID_QUIT,    COMMAND_ID_SIZE
};

// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE COMMANDS ********************************************
// ******************************************************************************************************************************
const CommandDescription motorFrictionIdentificationCommandDescr[COMMAND_ID_SIZE]  = 
{ 
//                  NAME            ID                          DESCRIPTION
CommandDescription("save",          COMMAND_ID_SAVE,            "Save the current identification state on text file"),
CommandDescription("help",          COMMAND_ID_HELP,            "Get instructions about how to communicate with this module"), 
CommandDescription("quit",          COMMAND_ID_QUIT,            "Stop the controller and quit the module")
};

}   // end namespace 

#endif
