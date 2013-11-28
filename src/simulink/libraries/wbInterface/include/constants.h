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

#ifndef LOCOMOTION_CONSTANTS
#define LOCOMOTION_CONSTANTS

#include <paramHelp/paramProxyBasic.h>
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

namespace locomotion
{

// When COMPUTE_WORLD_2_BASE_ROTOTRANSLATION is defined the controller computes the rototranslation 
// from world to floating base assuming that the left foot is always on the ground. Otherwise it 
// takes the rototranslation as an input streaming parameter (i.e. H_w2b).
#define COMPUTE_WORLD_2_BASE_ROTOTRANSLATION

/** List of available parameters of IOFormat constructor:
    precision       number of digits for floating point values, or one of the special constants StreamPrecision and FullPrecision.
    flags           either 0, or DontAlignCols, which allows to disable the alignment of columns, resulting in faster code.
    coeffSeparator  string printed between two coefficients of the same row
    rowSeparator    string printed between two rows
    rowPrefix       string printed at the beginning of each row
    rowSuffix       string printed at the end of each row
    matPrefix       string printed at the beginning of the matrix
    matSuffix       string printed at the end of the matrix */
static const IOFormat   matrixPrintFormat(1, DontAlignCols, " ", ";\n", "", "", "[", "]");

/** Types of printed messages */
enum MsgType {MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR};

// *** CONSTANTS
static const int        PRINT_PERIOD    = 3000;     // period of debug prints (in ms)
static const int        PRINT_MSG_LEVEL = MSG_DEBUG; // only messages whose type is >= PRINT_MSG_LEVEL are printed
static const double     KP_MAX          = 100.0;    // max value of proportional gains
static const double     DQ_MAX          = 1.0;      // max joint velocity allowed (rad/sec)
static const double     PINV_TOL        = 1e-4;     // threshold for truncated pseudoinverses

enum LocomotionSupportPhase
{ SUPPORT_DOUBLE, SUPPORT_LEFT, SUPPORT_RIGHT };     // foot support phase

// *** DEFAULT PARAMETER VALUES
static const string         DEFAULT_MODULE_NAME     = "locomotionControl";  
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
enum LocomotionParamId { 
    PARAM_ID_MODULE_NAME,       PARAM_ID_CTRL_PERIOD,       PARAM_ID_ROBOT_NAME,
    PARAM_ID_KP_COM,            PARAM_ID_KP_FOOT,           PARAM_ID_KP_POSTURE, 
    PARAM_ID_TRAJ_TIME_COM,     PARAM_ID_TRAJ_TIME_FOOT,    PARAM_ID_TRAJ_TIME_POSTURE,
    PARAM_ID_ACTIVE_JOINTS,     PARAM_ID_SUPPORT_PHASE,     PARAM_ID_PINV_DAMP,
    PARAM_ID_Q_MAX,             PARAM_ID_Q_MIN,             PARAM_ID_JNT_LIM_MIN_DIST,
    PARAM_ID_XDES_COM,          PARAM_ID_XDES_FOOT,         PARAM_ID_QDES,
    PARAM_ID_H_W2B,             
    PARAM_ID_X_COM,             PARAM_ID_XREF_COM,          PARAM_ID_X_FOOT,                
    PARAM_ID_XREF_FOOT,         PARAM_ID_Q,                 PARAM_ID_QREF,
    PARAM_ID_SIZE /*This is the number of parameters, so it must be the last value of the enum.*/
};

// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
// ******************************************************************************************************************************
const ParamProxyInterface *const locomotionParamDescr[PARAM_ID_SIZE] = 
{ 
//                          NAME                ID                          SIZE                        CONSTRAINTS                                 I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
new ParamProxyBasic<string>("name",             PARAM_ID_MODULE_NAME,       1,                          ParamConstraint<string>(),                  PARAM_CONFIG,       &DEFAULT_MODULE_NAME,           "Name of the instance of the module"),
new ParamProxyBasic<int>(   "period",           PARAM_ID_CTRL_PERIOD,       1,                          ParamBilatBounds<int>(1,1000),              PARAM_CONFIG,       &DEFAULT_CTRL_PERIOD,           "Period of the control loop (ms)"),
new ParamProxyBasic<string>("robot",            PARAM_ID_ROBOT_NAME,        1,                          ParamConstraint<string>(),                  PARAM_CONFIG,       &DEFAULT_ROBOT_NAME,            "Name of the robot"), 
// ************************************************* RPC PARAMETERS ****************************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("kp com",          PARAM_ID_KP_COM,             2,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KP_COM.data(),          "Proportional gain for the COM position control"), 
new ParamProxyBasic<double>("kp foot",         PARAM_ID_KP_FOOT,            6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KP_FOOT.data(),         "Proportional gain for the foot pose control"), 
new ParamProxyBasic<double>("kp posture",      PARAM_ID_KP_POSTURE,         ParamSize(ICUB_DOFS,true),  ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KP_POSTURE.data(),      "Proportional gain for the joint posture control"), 
new ParamProxyBasic<double>("tt com",          PARAM_ID_TRAJ_TIME_COM,      1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_TT_COM,                "Trajectory time for the COM minimum jerk trajectory generator"), 
new ParamProxyBasic<double>("tt foot",         PARAM_ID_TRAJ_TIME_FOOT,     1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_TT_FOOT,               "Trajectory time for the foot minimum jerk trajectory generator"), 
new ParamProxyBasic<double>("tt posture",      PARAM_ID_TRAJ_TIME_POSTURE,  1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_TT_POSTURE,            "Trajectory time for the posture minimum jerk trajectory generator"), 
new ParamProxyBasic<int>(   "active joints",   PARAM_ID_ACTIVE_JOINTS,      ICUB_DOFS,                  ParamBilatBounds<int>(0, 1),                PARAM_IN_OUT,       DEFAULT_ACTIVE_JNTS.data(),     "Selection of which joints are used in the control (1: active, 0: inactive)"), 
new ParamProxyBasic<double>("pinv damp",       PARAM_ID_PINV_DAMP,          1,                          ParamBilatBounds<double>(1e-8, 1.0),        PARAM_IN_OUT,       &DEFAULT_PINV_DAMP,             "Damping factor used in the pseudoinverses"),
new ParamProxyBasic<double>("q max",           PARAM_ID_Q_MAX,              ICUB_DOFS,                  ParamConstraint<double>(),                  PARAM_IN_OUT,       DEFAULT_Q_MAX.data(),           "Joint upper bounds"),
new ParamProxyBasic<double>("q min",           PARAM_ID_Q_MIN,              ICUB_DOFS,                  ParamConstraint<double>(),                  PARAM_IN_OUT,       DEFAULT_Q_MIN.data(),           "Joint lower bounds"),
new ParamProxyBasic<double>("jlmd",            PARAM_ID_JNT_LIM_MIN_DIST,   1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_JNT_LIM_MIN_DIST,      "Minimum distance to maintain from the joint limits"),
// ************************************************* STREAMING INPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<int>(   "support phase",   PARAM_ID_SUPPORT_PHASE,      1,                          ParamBilatBounds<int>(0, 2),                PARAM_IN_STREAM,    &DEFAULT_SUPPORT_PHASE,         "Foot support phase, 0: double, 1: left foot, 2: right foot"), 
new ParamProxyBasic<double>("xd com",          PARAM_ID_XDES_COM,           2,                          ParamBilatBounds<double>(-0.26, 0.1),       PARAM_IN_STREAM,    DEFAULT_XDES_COM.data(),        "Desired position of the center of mass"),
new ParamProxyBasic<double>("xd foot",         PARAM_ID_XDES_FOOT,          7,                          ParamBilatBounds<double>(-6.0, 6.0),        PARAM_IN_STREAM,    DEFAULT_XDES_FOOT.data(),       "Desired position/orientation of the swinging foot"),
new ParamProxyBasic<double>("qd",              PARAM_ID_QDES,               ICUB_DOFS,                  ParamBilatBounds<double>(-200.0, 200.0),    PARAM_IN_STREAM,    DEFAULT_QDES.data(),            "Desired joint angles"),
new ParamProxyBasic<double>("H_w2b",           PARAM_ID_H_W2B,              16,                         ParamBilatBounds<double>(-100.0, 100.0),    PARAM_IN_STREAM,    DEFAULT_H_W2B.data(),           "Estimated rototranslation matrix between world and robot base reference frames"),
// ************************************************* STREAMING OUTPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("x com",           PARAM_ID_X_COM,              2,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_OUT_STREAM,   DEFAULT_X_COM.data(),           "Position of the center of mass"),
new ParamProxyBasic<double>("x foot",          PARAM_ID_X_FOOT,             7,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_OUT_STREAM,   DEFAULT_X_FOOT.data(),          "Position/orientation of the swinging foot"),
new ParamProxyBasic<double>("q",               PARAM_ID_Q,                  ICUB_DOFS,                  ParamBilatBounds<double>(-100.0, 100.0),    PARAM_OUT_STREAM,   DEFAULT_Q.data(),               "Joint angles"),
new ParamProxyBasic<double>("xr com",          PARAM_ID_XREF_COM,           2,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_OUT_STREAM,   DEFAULT_XREF_COM.data(),        "Reference position of the center of mass generated by a min jerk trajectory generator"),
new ParamProxyBasic<double>("xr foot",         PARAM_ID_XREF_FOOT,          7,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_OUT_STREAM,   DEFAULT_XREF_FOOT.data(),       "Reference position/orientation of the swinging foot generated by a min jerk trajectory generator"),
new ParamProxyBasic<double>("qr",              PARAM_ID_QREF,               ICUB_DOFS,                  ParamBilatBounds<double>(-100.0, 100.0),    PARAM_OUT_STREAM,   DEFAULT_QREF.data(),            "Reference joint angles generated by a min jerk trajectory generator")
};



// *** IDs of all the module command
enum LocomotionCommandId { 
    COMMAND_ID_START,   COMMAND_ID_STOP,    COMMAND_ID_HELP,    COMMAND_ID_QUIT, 
    COMMAND_ID_SIZE
};
// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE COMMANDS ********************************************
// ******************************************************************************************************************************
const CommandDescription locomotionCommandDescr[COMMAND_ID_SIZE]  = 
{ 
//                  NAME            ID                          DESCRIPTION
CommandDescription("start",         COMMAND_ID_START,           "Start the controller"), 
CommandDescription("stop",          COMMAND_ID_STOP,            "Stop the controller"), 
CommandDescription("help",          COMMAND_ID_HELP,            "Get instructions about how to communicate with this module"), 
CommandDescription("quit",          COMMAND_ID_QUIT,            "Stop the controller and quit the module"), 
};

}

#endif
