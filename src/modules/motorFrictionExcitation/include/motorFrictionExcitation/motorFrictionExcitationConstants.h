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

#ifndef _MOTOR_FRICTION_EXCITATION_CONSTANTS
#define _MOTOR_FRICTION_EXCITATION_CONSTANTS

#include <paramHelp/paramProxyBasic.h>
#include <Eigen/Core>                               // import most common Eigen types
#include <vector>
#include <string>
#include <yarp/sig/Matrix.h>

using namespace paramHelp;
using namespace Eigen;
using namespace std;

static const int       ICUB_DOFS = 25;    // number of (the main) degrees of freedom of iCub

// define some types
typedef yarp::sig::Matrix                          MatrixY;    // to not mistake Yarp Matrix and Eigen Matrix
typedef Eigen::Matrix<double,1,1>                  Vector1d;
typedef Eigen::Matrix<double,6,1>                  Vector6d;
typedef Eigen::Matrix<double,7,1>                  Vector7d;
typedef Eigen::Matrix<double,ICUB_DOFS,1>          VectorNd;
typedef Eigen::Matrix<int,ICUB_DOFS,1>             VectorNi;
typedef Eigen::Matrix<double,6,Dynamic,RowMajor>   JacobianMatrix;     // a Jacobian is 6 rows and N columns

namespace motorFrictionExcitation
{
/** Types of printed messages */
enum MsgType {MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR};

///< specify whether or not the commands are sent to the motors
enum MFE_MotorCommandMode
{
    DO_NOT_SEND_COMMANDS_TO_MOTORS = 0,
    SEND_COMMANDS_TO_MOTORS = 1
};

// *** CONSTANTS
static const int        PRINT_PERIOD    = 1000;         ///< period of debug prints (in ms)
static const int        PRINT_MSG_LEVEL = MSG_DEBUG;    ///< only messages whose type is greater than or equal to PRINT_MSG_LEVEL are printed
static const double     PWM_MAX         = 300;          ///< max motor PWM allowed (in [-1333; 1333])
static const double     MAX_POS_INTEGRAL = 30.0;

// *** DEFAULT PARAMETER VALUES
static const string                 DEFAULT_MODULE_NAME     = "motorFrictionExcitationControl"; ///< name of the module 
static const int                    DEFAULT_CTRL_PERIOD     = 10;                               ///< controller period in ms
static const string                 DEFAULT_ROBOT_NAME      = "icubSim";                        ///< robot name
static const VectorNd               DEFAULT_Q_MAX           = VectorNd::Constant(150.0);
static const VectorNd               DEFAULT_Q_MIN           = VectorNd::Constant(-150.0);
static const int                    DEFAULT_SEND_COMMANDS   = SEND_COMMANDS_TO_MOTORS;
static const double                 DEFAULT_JNT_LIM_MIN_DIST = 10.0;
static const double                 DEFAULT_POS_INT_GAIN    = 1e-5;

// *** IDs of all the module command
enum MotorFrictionExcitationCommandId { 
    COMMAND_ID_START,   COMMAND_ID_STOP,    COMMAND_ID_HELP,    COMMAND_ID_QUIT, 
    COMMAND_ID_SIZE
};

// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE COMMANDS ********************************************
// ******************************************************************************************************************************
const CommandDescription motorFrictionExcitationCommandDescr[]  = 
{ 
//                  NAME            ID                          DESCRIPTION
CommandDescription("start",         COMMAND_ID_START,           "Start the controller"), 
CommandDescription("stop",          COMMAND_ID_STOP,            "Stop the controller"), 
CommandDescription("help",          COMMAND_ID_HELP,            "Get instructions about how to communicate with this module"), 
CommandDescription("quit",          COMMAND_ID_QUIT,            "Stop the controller and quit the module"), 
};

}   // end namespace 

#endif
