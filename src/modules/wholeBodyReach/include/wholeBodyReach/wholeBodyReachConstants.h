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

#ifndef WHOLE_BODY_REACH_CONSTANTS
#define WHOLE_BODY_REACH_CONSTANTS

#include <paramHelp/paramProxyBasic.h>
#include <Eigen/Core>                               // import most common Eigen types
#include <vector>
#include <string>
#include <yarp/sig/Matrix.h>
#include <wholeBodyReach/Stopwatch.h>

using namespace paramHelp;
using namespace std;

static const int       ICUB_DOFS = 25;    // number of (the main) degrees of freedom of iCub

namespace Eigen
{
    typedef Matrix<double,6,1>                      Vector6d;
    typedef Matrix<double,7,1>                      Vector7d;
    typedef Matrix<double,ICUB_DOFS,1>              VectorNd;
    typedef Matrix<int,ICUB_DOFS,1>                 VectorNi;
    typedef Matrix<double,6,Dynamic,RowMajor>       JacobianMatrix;     // a Jacobian is 6 rows and N columns
    
    typedef Matrix<double,Dynamic,Dynamic,RowMajor> MatrixRXd;     /// Dynamic matrix with row-major storage order
    typedef Matrix<double,3,3,RowMajor>             MatrixR3d;
    typedef Matrix<double,4,4,RowMajor>             MatrixR4d;
    typedef Matrix<double,6,6,RowMajor>             MatrixR6d;
    
    typedef Ref<VectorXd>                           VectorRef;      /// Type used to pass Eigen vectors by reference
    typedef Ref<MatrixRXd>                          MatrixRef;      /// Type used to pass Eigen matrices by reference
    typedef const Ref<const VectorXd>&              VectorConst;    /// Type used to pass Eigen vectors by const reference
    typedef const Ref<const MatrixRXd>&             MatrixConst;    /// Type used to pass Eigen matrices by const reference
    
    typedef JacobiSVD<MatrixRXd>                    SVD;            /// svd of RowMajor matrix
}

// define some types
typedef yarp::sig::Matrix                               MatrixY;    // to not mistake Yarp Matrix and Eigen Matrix

namespace wholeBodyReach
{
    
// if DO_PROFILING is defined then the profiling is active
#define DO_PROFILING

#ifdef DO_PROFILING
    #define START_PROFILING(name)   getProfiler().start(name)
    #define STOP_PROFILING(name)    getProfiler().stop(name)
    #define PRINT_PROFILING_INFO    getProfiler().report_all()
#else
    #define START_PROFILING(name)
    #define STOP_PROFILING(name)
    #define PRINT_PROFILING_INFO
#endif

// When COMPUTE_WORLD_2_BASE_ROTOTRANSLATION is defined the controller computes the rototranslation
// from world to floating base assuming that the left foot is always on the ground. Otherwise it 
// takes the homogeneous transformation as an input streaming parameter (i.e. H_w2b).
#define COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    
// When DO_NOT_USE_WHOLE_BODY_STATE_INTERFACE is defined the controller does not use the methods
// of the iWholeBodyState class. Instead, it uses iWholeBodySensor to read the sensor and then
// it performs some state estimation to get the desired information
#define DO_NOT_USE_WHOLE_BODY_STATE_INTERFACE
    
struct ContactPlaneSize
{
    double          xPos; /// size of the contact plane in positive x direction
    double          xNeg; /// size of the contact plane in negative x direction
    double          yPos; /// size of the contact plane in positive y direction
    double          yNeg; /// size of the contact plane in negative y direction
    
    ContactPlaneSize(double xp, double xn, double yp, double yn)
    :xPos(xp), xNeg(xn), yPos(yp), yNeg(yn) {}
};

/** List of available parameters of IOFormat constructor:
    precision       number of digits for floating point values, or one of the special constants StreamPrecision and FullPrecision.
    flags           either 0, or DontAlignCols, which allows to disable the alignment of columns, resulting in faster code.
    coeffSeparator  string printed between two coefficients of the same row
    rowSeparator    string printed between two rows
    rowPrefix       string printed at the beginning of each row
    rowSuffix       string printed at the end of each row
    matPrefix       string printed at the beginning of the matrix
    matSuffix       string printed at the end of the matrix */
static const Eigen::IOFormat matrixPrintFormat(1, Eigen::DontAlignCols, " ", ";\n", "", "", "[", "]");

/** Types of printed messages */
//enum MsgType {MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR};

// *** CONSTANTS
static const double     WBR_DEG2RAD     = 3.14/180.0;
static const double     WBR_RAD2DEG     = 180.0/3.14;
static const double     REAL_TIME_FACTOR = 1.00;    // simulation real-time factor
static const double     PRINT_PERIOD    = 2.0;      // period of debug prints (in sec)
//static const int        PRINT_MSG_LEVEL = MSG_DEBUG; // only messages whose type is >= PRINT_MSG_LEVEL are printed
static const double     KP_MAX          = 100.0;    // max value of proportional gains
static const double     DQ_MAX          = 1.0;      // max joint velocity allowed (rad/sec)
static const double     TAU_MAX         = 200.0;    // max joint torque allowed (Nm)
static const double     PINV_TOL        = 1e-4;     // threshold for truncated pseudoinverses
static const double     FORCE_NORMAL_MIN = 5.0;
static const double     FORCE_NORMAL_MAX = 300.0;

static const string     GRASP_HAND_LINK_NAME        = "r_gripper";
static const string     SUPPORT_FOREARM_LINK_NAME   = "l_forearm";
static const string     LEFT_FOOT_LINK_NAME         = "l_sole";
static const string     RIGHT_FOOT_LINK_NAME        = "r_sole";
//static const ContactPlaneSize ICUB_FOOT_SIZE(0.1,0.01,0.05,0.05);
static const ContactPlaneSize ICUB_FOOT_SIZE(0.1,0.1,0.1,0.1);

enum WholeBodyReachSupportPhase
{
    WBR_SUPPORT_DOUBLE,     /// the two feet
    WBR_SUPPORT_TRIPLE      /// the two feet + the supporting forearm
};
    
enum WholeBodyReachPhase
{
    WBR_OFF,                /// no control (i.e. position control)
    WBR_GO_HOME,            /// move to home position
    WBR_CONTACT_APPROACH,   /// move the forearm to the contact point
    WBR_CONTACT_LOAD,       /// increase the contact force on the forearm and move CoM
    WBR_REACH,              /// reach the target with the free hand
    WBR_GRASP,              /// grasp the target
    WBR_GO_BACK             /// move the free hand back
};

// *** DEFAULT PARAMETER VALUES
static const Eigen::Vector3d       ZERO_3D                 = Eigen::Vector3d::Constant(0.0);
static const Eigen::Vector6d       ZERO_6D                 = Eigen::Vector6d::Constant(0.0);
static const Eigen::Vector7d       ZERO_7D                 = Eigen::Vector7d::Constant(0.0);
static const Eigen::VectorNd       ZERO_ND                 = Eigen::VectorNd::Constant(0.0);
static const Eigen::Vector3d       ONE_3D                  = Eigen::Vector3d::Constant(1.0);
static const Eigen::Vector6d       ONE_6D                  = Eigen::Vector6d::Constant(1.0);
static const Eigen::Vector7d       ONE_7D                  = Eigen::Vector7d::Constant(1.0);

static const string             DEFAULT_MODULE_NAME = "wholeBodyReach";
static const int                DEFAULT_CTRL_PERIOD = 10;                   // controller period in ms
static const string             DEFAULT_ROBOT_NAME  = "icubGazeboSim";            // robot name
static const Eigen::Vector6d    DEFAULT_KP_MOMENTUM = Eigen::Vector6d::Constant(1.0);
static const Eigen::Vector6d    DEFAULT_KP_FOREARM  = Eigen::Vector6d::Constant(1.0);
static const Eigen::Vector6d    DEFAULT_KP_HAND     = Eigen::Vector6d::Constant(1.0);
static const Eigen::VectorNd    DEFAULT_KP_POSTURE  = Eigen::VectorNd::Constant(1.0);
static const Eigen::Vector6d    DEFAULT_KD_MOMENTUM = Eigen::Vector6d::Constant(2.0);
static const Eigen::Vector6d    DEFAULT_KD_FOREARM  = Eigen::Vector6d::Constant(2.0);
static const Eigen::Vector6d    DEFAULT_KD_HAND     = Eigen::Vector6d::Constant(2.0);
static const Eigen::VectorNd    DEFAULT_KD_POSTURE  = Eigen::VectorNd::Constant(2.0);
static const double             DEFAULT_TT_MOMENTUM = 4.0;
static const double             DEFAULT_TT_FOREARM  = 4.0;
static const double             DEFAULT_TT_HAND     = 4.0;
static const double             DEFAULT_TT_POSTURE  = 4.0;
static const int                DEFAULT_SUPPORT_PHASE       = WBR_SUPPORT_DOUBLE;
static const double             DEFAULT_NUM_DAMP            = 1e-4;
static const int                DEFAULT_USE_NULLSPACE_BASE  = 0;   /// true: solver uses basis, false: it uses projectors
static const Eigen::VectorNd    DEFAULT_Q_MAX               = Eigen::VectorNd::Constant(180.0);
static const Eigen::VectorNd    DEFAULT_Q_MIN               = Eigen::VectorNd::Constant(-180.0);
static const double             DEFAULT_JNT_LIM_MIN_DIST    = 5.0;
static const double             DEFAULT_FORCE_FRICTION      = 0.5;  // friction cone coefficient for tangential forces
static const double             DEFAULT_MOMENT_FRICTION     = 0.5;  // friction cone coefficient for normal moment
// Streaming parameters
static const Eigen::Vector3d       DEFAULT_XDES_COM        = Eigen::Vector3d::Constant(0.0);
static const Eigen::Vector7d       DEFAULT_XDES_FOREARM    = Eigen::Vector7d::Constant(0.0);
static const Eigen::Vector7d       DEFAULT_XDES_HAND       = Eigen::Vector7d::Constant(0.0);
static const Eigen::Matrix4d       DEFAULT_H_W2B           = Eigen::Matrix4d::Identity();

// *** IDs of all the module parameters
enum WholeBodyReachParamId { 
    PARAM_ID_MODULE_NAME,           PARAM_ID_CTRL_PERIOD,       PARAM_ID_ROBOT_NAME,
    
    PARAM_ID_KP_MOMENTUM,           PARAM_ID_KP_FOREARM,
    PARAM_ID_KP_HAND,               PARAM_ID_KP_POSTURE,
    PARAM_ID_KP_CONSTRAINTS,
    PARAM_ID_KD_MOMENTUM,           PARAM_ID_KD_FOREARM,
    PARAM_ID_KD_HAND,               PARAM_ID_KD_POSTURE,
    PARAM_ID_TRAJ_TIME_MOMENTUM,    PARAM_ID_TRAJ_TIME_FOREARM,
    PARAM_ID_TRAJ_TIME_HAND,        PARAM_ID_TRAJ_TIME_POSTURE,
    PARAM_ID_SUPPORT_PHASE,         PARAM_ID_NUM_DAMP,          PARAM_ID_USE_NULLSPACE_BASE,
    PARAM_ID_Q_MAX,                 PARAM_ID_Q_MIN,             PARAM_ID_JNT_LIM_MIN_DIST,
    PARAM_ID_FORCE_FRICTION,        PARAM_ID_MOMENT_FRICTION,   PARAM_ID_WRENCH_WEIGHTS,
    
    PARAM_ID_XDES_COM,              PARAM_ID_XDES_FOREARM,
    PARAM_ID_XDES_HAND,             PARAM_ID_QDES,
    PARAM_ID_H_W2B,             
    
    PARAM_ID_X_COM,                 PARAM_ID_DX_COM,
    PARAM_ID_XREF_COM,              PARAM_ID_MOMENTUM,
    PARAM_ID_X_FOREARM,             PARAM_ID_XREF_FOREARM,
    PARAM_ID_X_HAND,                PARAM_ID_XREF_HAND,
    PARAM_ID_X_BASE,                PARAM_ID_V_BASE,
    PARAM_ID_Q,                     PARAM_ID_DQ,                PARAM_ID_QREF,
    PARAM_ID_FORCE_INEQ_R_FOOT,     PARAM_ID_FORCE_INEQ_L_FOOT, PARAM_ID_FORCE_INEQ_FOREARM,
    PARAM_ID_SIZE /*This is the number of parameters, so it must be the last value of the enum.*/
};

// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
// ******************************************************************************************************************************
const ParamProxyInterface *const wholeBodyReachParamDescr[PARAM_ID_SIZE] = 
{ 
//                          NAME                    ID                          SIZE                        CONSTRAINTS                                 I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
new ParamProxyBasic<string>("name",                 PARAM_ID_MODULE_NAME,       1,                          ParamConstraint<string>(),                  PARAM_CONFIG,       &DEFAULT_MODULE_NAME,           "Name of the instance of the module"),
new ParamProxyBasic<int>(   "period",               PARAM_ID_CTRL_PERIOD,       1,                          ParamBilatBounds<int>(1,1000),              PARAM_CONFIG,       &DEFAULT_CTRL_PERIOD,           "Period of the control loop (ms)"),
new ParamProxyBasic<string>("robot",                PARAM_ID_ROBOT_NAME,        1,                          ParamConstraint<string>(),                  PARAM_CONFIG,       &DEFAULT_ROBOT_NAME,            "Name of the robot"),

// ************************************************* RPC PARAMETERS ****************************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("kp momentum",          PARAM_ID_KP_MOMENTUM,       6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KP_MOMENTUM.data(),     "Proportional gain for the momentum control"),
new ParamProxyBasic<double>("kp forearm",           PARAM_ID_KP_FOREARM,        6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KP_FOREARM.data(),      "Proportional gain for the forearm control"),
new ParamProxyBasic<double>("kp hand",              PARAM_ID_KP_HAND,           6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KP_HAND.data(),         "Proportional gain for the hand control"),
new ParamProxyBasic<double>("kp posture",           PARAM_ID_KP_POSTURE,        ParamSize(ICUB_DOFS,true),  ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KP_POSTURE.data(),      "Proportional gain for the joint posture control"),
new ParamProxyBasic<double>("kp constraints",       PARAM_ID_KP_CONSTRAINTS,    6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       ONE_6D.data(),                  "Proportional gain for correcting constraint drifts"),
new ParamProxyBasic<double>("kd momentum",          PARAM_ID_KD_MOMENTUM,       6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KD_MOMENTUM.data(),     "Derivative gain for the momentum control"),
new ParamProxyBasic<double>("kd forearm",           PARAM_ID_KD_FOREARM,        6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KD_FOREARM.data(),      "Derivative gain for the forearm control"),
new ParamProxyBasic<double>("kd hand",              PARAM_ID_KD_HAND,           6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KD_HAND.data(),         "Derivative gain for the hand control"),
new ParamProxyBasic<double>("kd posture",           PARAM_ID_KD_POSTURE,        ParamSize(ICUB_DOFS,true),  ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KD_POSTURE.data(),      "Derivative gain for the joint posture control"),
new ParamProxyBasic<double>("tt momentum",          PARAM_ID_TRAJ_TIME_MOMENTUM,1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_TT_MOMENTUM,           "Trajectory time for the momentum minimum jerk trajectory generator"),
new ParamProxyBasic<double>("tt forearm",           PARAM_ID_TRAJ_TIME_FOREARM, 1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_TT_FOREARM,            "Trajectory time for the forearm minimum jerk trajectory generator"),
new ParamProxyBasic<double>("tt hand",              PARAM_ID_TRAJ_TIME_HAND,    1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_TT_HAND,               "Trajectory time for the hand minimum jerk trajectory generator"),
new ParamProxyBasic<double>("tt posture",           PARAM_ID_TRAJ_TIME_POSTURE, 1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_TT_POSTURE,            "Trajectory time for the posture minimum jerk trajectory generator"),
new ParamProxyBasic<double>("num damp",             PARAM_ID_NUM_DAMP,          1,                          ParamBilatBounds<double>(1e-9, 1.0),        PARAM_IN_OUT,       &DEFAULT_NUM_DAMP,              "Numerical damping used to regularize the optimization problems"),
new ParamProxyBasic<int>(   "use nullspace base",   PARAM_ID_USE_NULLSPACE_BASE,1,                          ParamBilatBounds<int>(0, 1),                PARAM_IN_OUT,       &DEFAULT_USE_NULLSPACE_BASE,    "0: use nullspace projectors, 1: use nullspace basis"),
new ParamProxyBasic<double>("q max",                PARAM_ID_Q_MAX,             ICUB_DOFS,                  ParamConstraint<double>(),                  PARAM_IN_OUT,       DEFAULT_Q_MAX.data(),           "Joint upper bounds"),
new ParamProxyBasic<double>("q min",                PARAM_ID_Q_MIN,             ICUB_DOFS,                  ParamConstraint<double>(),                  PARAM_IN_OUT,       DEFAULT_Q_MIN.data(),           "Joint lower bounds"),
new ParamProxyBasic<double>("jlmd",                 PARAM_ID_JNT_LIM_MIN_DIST,  1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_JNT_LIM_MIN_DIST,      "Minimum distance to maintain from the joint limits"),
new ParamProxyBasic<double>("force friction",       PARAM_ID_FORCE_FRICTION,    1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_FORCE_FRICTION,        "Friciton coefficient for tangential forces"),
new ParamProxyBasic<double>("moment friction",      PARAM_ID_MOMENT_FRICTION,   1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_MOMENT_FRICTION,       "Friciton coefficient for normal moments"),
new ParamProxyBasic<double>("wrench weights",       PARAM_ID_WRENCH_WEIGHTS,    6,                          ParamBilatBounds<double>(1e-4, 1e4),        PARAM_IN_OUT,       ONE_6D.data(),                  "Weights used to penalize 6d wrenches"),
    
// ************************************************* STREAMING INPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<int>(   "support phase",        PARAM_ID_SUPPORT_PHASE,     1,                          ParamBilatBounds<int>(0, 2),                PARAM_IN_STREAM,    &DEFAULT_SUPPORT_PHASE,         "Contact support phase, 0: double, 1: triple"),
new ParamProxyBasic<double>("xd com",               PARAM_ID_XDES_COM,          3,                          ParamBilatBounds<double>(-0.26, 1.0),       PARAM_IN_STREAM,    DEFAULT_XDES_COM.data(),        "Desired 3d position of the center of mass"),
new ParamProxyBasic<double>("xd forearm",           PARAM_ID_XDES_FOREARM,      7,                          ParamBilatBounds<double>(-6.0, 6.0),        PARAM_IN_STREAM,    DEFAULT_XDES_FOREARM.data(),    "Desired position/orientation of the forearm"),
new ParamProxyBasic<double>("xd hand",              PARAM_ID_XDES_HAND,         7,                          ParamBilatBounds<double>(-6.0, 6.0),        PARAM_IN_STREAM,    DEFAULT_XDES_HAND.data(),       "Desired position/orientation of the grasping hand"),
new ParamProxyBasic<double>("qd",                   PARAM_ID_QDES,              ICUB_DOFS,                  ParamBilatBounds<double>(-200.0, 200.0),    PARAM_IN_STREAM,    ZERO_ND.data(),                 "Desired joint angles"),
new ParamProxyBasic<double>("H_w2b",                PARAM_ID_H_W2B,             16,                         ParamBilatBounds<double>(-100.0, 100.0),    PARAM_IN_STREAM,    DEFAULT_H_W2B.data(),           "Estimated rototranslation matrix between world and robot base reference frames"),
    
// ************************************************* STREAMING OUTPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("x com",                PARAM_ID_X_COM,             3,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_MONITOR,      ZERO_3D.data(),                 "3d Position of the center of mass"),
new ParamProxyBasic<double>("dx com",               PARAM_ID_DX_COM,            3,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_MONITOR,      ZERO_3D.data(),                 "3d velocity of the center of mass"),
new ParamProxyBasic<double>("xr com",               PARAM_ID_XREF_COM,          3,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_MONITOR,      ZERO_3D.data(),                 "3d Reference position of the center of mass generated by a min jerk trajectory generator"),
new ParamProxyBasic<double>("momentum",             PARAM_ID_MOMENTUM,          6,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_MONITOR,      ZERO_6D.data(),                 "6d centroidal momentum"),
new ParamProxyBasic<double>("x forearm",            PARAM_ID_X_FOREARM,         7,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_MONITOR,      ZERO_7D.data(),                 "Position/orientation of the forearm"),
new ParamProxyBasic<double>("xr forearm",           PARAM_ID_XREF_FOREARM,      3,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_MONITOR,      ZERO_3D.data(),                 "Reference position/orientation of the forearm generated by a min jerk trajectory generator"),
new ParamProxyBasic<double>("x hand",               PARAM_ID_X_HAND,            7,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_MONITOR,      ZERO_7D.data(),                 "Position/orientation of the grasping hand"),
new ParamProxyBasic<double>("xr hand",              PARAM_ID_XREF_HAND,         3,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_MONITOR,      ZERO_3D.data(),                 "Reference position/orientation of the grasping hand generated by a min jerk trajectory generator"),
new ParamProxyBasic<double>("x base",               PARAM_ID_X_BASE,            7,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_MONITOR,      ZERO_7D.data(),                 "Position/orientation of the floating base"),
new ParamProxyBasic<double>("v base",               PARAM_ID_V_BASE,            6,                          ParamBilatBounds<double>(-10.0, 10.0),      PARAM_MONITOR,      ZERO_7D.data(),                 "Linear/angular velocity of the floating base"),
new ParamProxyBasic<double>("q",                    PARAM_ID_Q,                 ICUB_DOFS,                  ParamBilatBounds<double>(-100.0, 100.0),    PARAM_MONITOR,      ZERO_ND.data(),                 "Joint angles"),
new ParamProxyBasic<double>("dq",                   PARAM_ID_DQ,                ICUB_DOFS,                  ParamBilatBounds<double>(-100.0, 100.0),    PARAM_MONITOR,      ZERO_ND.data(),                 "Joint velocities"),
new ParamProxyBasic<double>("qr",                   PARAM_ID_QREF,              ICUB_DOFS,                  ParamBilatBounds<double>(-100.0, 100.0),    PARAM_MONITOR,      ZERO_ND.data(),                 "Reference joint angles generated by a min jerk trajectory generator"),
new ParamProxyBasic<double>("f inequalities rfoot",     PARAM_ID_FORCE_INEQ_R_FOOT, 6,                      ParamConstraint<double>(),                  PARAM_MONITOR,      ZERO_6D.data(),                 "1-2 tang/norm force, 3 norm force, 4-5 ZMP, 6 normal moment"),
new ParamProxyBasic<double>("f inequalities rfoot",     PARAM_ID_FORCE_INEQ_L_FOOT, 6,                      ParamConstraint<double>(),                  PARAM_MONITOR,      ZERO_6D.data(),                 "1-2 tang/norm force, 3 norm force, 4-5 ZMP, 6 normal moment"),
new ParamProxyBasic<double>("f inequalities forearm",   PARAM_ID_FORCE_INEQ_FOREARM,3,                      ParamConstraint<double>(),                  PARAM_MONITOR,      ZERO_3D.data(),                 "1-2 tang/norm force, 3 norm force, 4-5 ZMP, 6 normal moment")
};



// *** IDs of all the module command
enum WholeBodyReachCommandId { 
    COMMAND_ID_START,   COMMAND_ID_STOP,    COMMAND_ID_HELP,    COMMAND_ID_QUIT,
    COMMAND_ID_RESET_PROFILER,
    COMMAND_ID_SIZE
};
// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE COMMANDS ********************************************
// ******************************************************************************************************************************
const CommandDescription wholeBodyReachCommandDescr[COMMAND_ID_SIZE]  = 
{ 
//                  NAME                ID                          DESCRIPTION
CommandDescription("start",             COMMAND_ID_START,           "Start the controller"),
CommandDescription("stop",              COMMAND_ID_STOP,            "Stop the controller"),
CommandDescription("reset profiler",    COMMAND_ID_RESET_PROFILER,  "Reset the profiling statistics"),
CommandDescription("help",              COMMAND_ID_HELP,            "Get instructions about how to communicate with this module"),
CommandDescription("quit",              COMMAND_ID_QUIT,            "Stop the controller and quit the module"), 
};

}

#endif
