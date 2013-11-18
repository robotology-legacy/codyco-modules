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

#include <paramHelp/paramHelp.h>
#include <Eigen/Core>                               // import most common Eigen types
#include <vector>
#include <string>

using namespace paramHelp;
using namespace Eigen;
using namespace std;

static const int       ICUB_DOFS = 25;    // number of (the main) degrees of freedom of iCub

// define some types
typedef yarp::sig::Matrix                          MatrixY;    // to not mistake Yarp Matrix and Eigen Matrix
typedef Eigen::Matrix<double,6,1>                  Vector6d;
typedef Eigen::Matrix<double,7,1>                  Vector7d;
typedef Eigen::Matrix<double,ICUB_DOFS,1>          VectorNd;
typedef Eigen::Matrix<int,ICUB_DOFS,1>             VectorNi;
typedef Eigen::Matrix<double,6,Dynamic,RowMajor>   JacobianMatrix;     // a Jacobian is 6 rows and N columns

namespace motorFrictionExcitation
{

/** Types of printed messages */
enum MsgType {MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR};

// *** CONSTANTS
static const int        PRINT_PERIOD    = 3000;     // period of debug prints (in ms)
static const int        PRINT_MSG_LEVEL = MSG_DEBUG; // only messages whose type is >= PRINT_MSG_LEVEL are printed
static const double     KP_MAX          = 100.0;    // max value of proportional gains
static const double     DQ_MAX          = 1.0;      // max joint velocity allowed (rad/sec)
static const double     PINV_TOL        = 1e-4;     // threshold for truncated pseudoinverses

enum MotorFrictionExcitationSupportPhase
{ SUPPORT_DOUBLE, SUPPORT_LEFT, SUPPORT_RIGHT };     // foot support phase

// *** DEFAULT PARAMETER VALUES
static const string         DEFAULT_MODULE_NAME     = "motorFrictionExcitationControl";  
static const int            DEFAULT_CTRL_PERIOD     = 10;                   // controller period in ms
static const string         DEFAULT_ROBOT_NAME      = "icubSim";            // robot name
static const Vector2d       DEFAULT_KP_COM          = Vector2d::Constant(1.0);
static const Vector6d       DEFAULT_KP_FOOT         = Vector6d::Constant(1.0);
static const VectorNd       DEFAULT_KP_POSTURE      = VectorNd::Constant(1.0);
static const double         DEFAULT_TT_COM          = 4.0;
static const double         DEFAULT_TT_FOOT         = 4.0;
static const double         DEFAULT_TT_POSTURE      = 4.0;
static const VectorNi       DEFAULT_ACTIVE_JNTS     = VectorNi::Constant(1);
static const int            DEFAULT_SUPPORT_PHASE   = SUPPORT_DOUBLE;
static const double         DEFAULT_PINV_DAMP       = 1e-4;
static const VectorNd       DEFAULT_Q_MAX           = VectorNd::Constant(150.0);
static const VectorNd       DEFAULT_Q_MIN           = VectorNd::Constant(-150.0);
static const double         DEFAULT_JNT_LIM_MIN_DIST = 5.0;
// Streaming parameters
static const Vector2d       DEFAULT_XDES_COM        = Vector2d::Constant(0.0);
static const Vector7d       DEFAULT_XDES_FOOT       = Vector7d::Constant(0.0);
static const VectorNd       DEFAULT_QDES            = VectorNd::Constant(0.0);
static const Matrix4d       DEFAULT_H_W2B           = Matrix4d::Identity();
static const Vector2d       DEFAULT_XREF_COM        = Vector2d::Constant(0.0);
static const Vector7d       DEFAULT_XREF_FOOT       = Vector7d::Constant(0.0);
static const VectorNd       DEFAULT_QREF            = VectorNd::Constant(0.0);
static const Vector2d       DEFAULT_X_COM           = Vector2d::Constant(0.0);
static const Vector7d       DEFAULT_X_FOOT          = Vector7d::Constant(0.0);
static const VectorNd       DEFAULT_Q               = VectorNd::Constant(0.0);

// *** IDs of all the module parameters
enum MotorFrictionExcitationParamId { 
    PARAM_ID_MODULE_NAME,       PARAM_ID_CTRL_PERIOD,       PARAM_ID_ROBOT_NAME,
    PARAM_ID_JOINT_ID,          PARAM_ID_INIT_Q,            PARAM_ID_A,
    PARAM_ID_A0,                PARAM_ID_W,                 PARAM_ID_JOINT_LIM_THR,
    PARAM_ID_FRIC_PAR_COV_THR,  PARAM_FREE_MOTION_EXCIT,
    PARAM_ID_Q_MAX,             PARAM_ID_Q_MIN,             PARAM_ID_JNT_LIM_MIN_DIST,
    PARAM_ID_Q,
    PARAM_ID_SIZE /*This is the number of parameters, so it must be the last value of the enum.*/
};

// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
// ******************************************************************************************************************************
struct FreeMotionExcitation
{
    int jointId;
    VectorXd initialJointConfiguration;
};

const ParamDescription freeMotionExcitationParamDescr[] =
{
//               NAME                           ID                          TYPE                SIZE                        BOUNDS                              I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
ParamDescription("joint id",                    PARAM_ID_JOINT_ID,          PARAM_DATA_INT,     PARAM_SIZE_FREE,            ParamBounds(0,30),                  PARAM_CONFIG,       0,                              "Id(s) of the joint(s) to excite"),          
ParamDescription("initial joint configuration", PARAM_ID_INIT_Q,            PARAM_DATA_FLOAT,   PARAM_SIZE_FREE,            ParamBounds(-360,360),              PARAM_CONFIG,       0,                              "Id(s) of the joint(s) to excite"),
ParamDescription("a",                           PARAM_ID_A,                 PARAM_DATA_FLOAT,   PARAM_SIZE_FREE,            ParamBounds(0,10),                  PARAM_CONFIG,       0,                              "Id(s) of the joint(s) to excite"),
ParamDescription("a0",                          PARAM_ID_A0,                PARAM_DATA_FLOAT,   PARAM_SIZE_FREE,            ParamBounds(0,100),                 PARAM_CONFIG,       0,                              "Id(s) of the joint(s) to excite"),
ParamDescription("w",                           PARAM_ID_W,                 PARAM_DATA_FLOAT,   PARAM_SIZE_FREE,            ParamBounds(0,5),                   PARAM_CONFIG,       0,                              "Id(s) of the joint(s) to excite"),
ParamDescription("joint limit thresh",          PARAM_ID_JOINT_LIM_THR,     PARAM_DATA_FLOAT,   PARAM_SIZE_FREE,            ParamBounds(1,100),                 PARAM_CONFIG,       0,                              "Id(s) of the joint(s) to excite"),
ParamDescription("fric param covar thresh",     PARAM_ID_FRIC_PAR_COV_THR,  PARAM_DATA_FLOAT,   PARAM_SIZE_FREE,            ParamBounds::createLowerBound(0),   PARAM_CONFIG,       0,                              "Id(s) of the joint(s) to excite")
};

const ParamDescription motorFrictionExcitationParamDescr[]  = 
{
// ************************************************* STRUCT PARAMETERS ****************************************************************************************************************************************************************************************************************************************
//               NAME                       ID                          SIZE                SUBPARAMETERS                   SUBPARAMETER NUMBER     DESCRIPTION
ParamDescription("free motion excitation",  PARAM_FREE_MOTION_EXCIT,    PARAM_SIZE_FREE,    freeMotionExcitationParamDescr, 7,                      "Information describing the excitation process for the joints to move in free motion"),
// ************************************************* SIMPLE PARAMETERS ****************************************************************************************************************************************************************************************************************************************
//               NAME                       ID                          TYPE                SIZE                        BOUNDS                              I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
ParamDescription("name",                    PARAM_ID_MODULE_NAME,       PARAM_DATA_STRING,  1,                          PARAM_BOUNDS_INF,                   PARAM_CONFIG,       &DEFAULT_MODULE_NAME,           "Name of the instance of the module"), 
ParamDescription("period",                  PARAM_ID_CTRL_PERIOD,       PARAM_DATA_INT,     1,                          ParamBounds(1, 1000),               PARAM_CONFIG,       &DEFAULT_CTRL_PERIOD,           "Period of the control loop (ms)"), 
ParamDescription("robot",                   PARAM_ID_ROBOT_NAME,        PARAM_DATA_STRING,  1,                          PARAM_BOUNDS_INF,                   PARAM_CONFIG,       &DEFAULT_ROBOT_NAME,            "Name of the robot"), 
// ************************************************* RPC PARAMETERS ****************************************************************************************************************************************************************************************************************************************
ParamDescription("q max",                   PARAM_ID_Q_MAX,             PARAM_DATA_FLOAT,   ICUB_DOFS,                  PARAM_BOUNDS_INF,                   PARAM_IN_OUT,       DEFAULT_Q_MAX.data(),           "Joint upper bounds"),
ParamDescription("q min",                   PARAM_ID_Q_MIN,             PARAM_DATA_FLOAT,   ICUB_DOFS,                  PARAM_BOUNDS_INF,                   PARAM_IN_OUT,       DEFAULT_Q_MIN.data(),           "Joint lower bounds"),
ParamDescription("jlmd",                    PARAM_ID_JNT_LIM_MIN_DIST,  PARAM_DATA_FLOAT,   1,                          ParamBounds(0.1, PARAM_BOUND_INF),  PARAM_IN_OUT,       &DEFAULT_JNT_LIM_MIN_DIST,      "Minimum distance to maintain from the joint limits"),
// ************************************************* STREAMING INPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
// ************************************************* STREAMING OUTPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
ParamDescription("q",                       PARAM_ID_Q,                 PARAM_DATA_FLOAT,   ICUB_DOFS,                  ParamBounds(-100.0, 100.0),         PARAM_OUT_STREAM,   DEFAULT_Q.data(),               "Joint angles"),

};





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

}

#endif
