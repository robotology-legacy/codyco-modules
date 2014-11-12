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

#ifndef WHOLE_BODY_REACH_PARAMS
#define WHOLE_BODY_REACH_PARAMS

#include <paramHelp/paramProxyBasic.h>
#include <Eigen/Core>                               // import most common Eigen types
#include <vector>
#include <string>
#include "wholeBodyReach/wholeBodyReachUtils.h"

using namespace paramHelp;
using namespace std;

namespace wholeBodyReach
{
    
// *** DEFAULT PARAMETER VALUES
static const Eigen::Vector3d       ZERO_3D                 = Eigen::Vector3d::Constant(0.0);
static const Eigen::Vector6d       ZERO_6D                 = Eigen::Vector6d::Constant(0.0);
static const Eigen::Vector7d       ZERO_7D                 = Eigen::Vector7d::Constant(0.0);
static const Eigen::VectorNd       ZERO_ND                 = Eigen::VectorNd::Constant(0.0);
static const Eigen::Vector3d       ONE_3D                  = Eigen::Vector3d::Constant(1.0);
static const Eigen::Vector6d       ONE_6D                  = Eigen::Vector6d::Constant(1.0);
static const Eigen::Vector7d       ONE_7D                  = Eigen::Vector7d::Constant(1.0);

static const string             DEFAULT_MODULE_NAME     = "wholeBodyReach";
static const int                DEFAULT_CTRL_PERIOD     = 10;                   // controller period in ms
static const string             DEFAULT_ROBOT_NAME      = "icubGazeboSim";            // robot name
static const int                DEFAULT_CTRL_ALG        = WBR_CTRL_ALG_COM_SOT;
static const int                DEFAULT_INTEGRATE_EOM   = 0;
static const Eigen::Vector6d    DEFAULT_KP_MOMENTUM     = Eigen::Vector6d::Constant(1.0);
static const Eigen::Vector6d    DEFAULT_KP_FOREARM      = Eigen::Vector6d::Constant(1.0);
static const Eigen::Vector6d    DEFAULT_KP_HAND         = Eigen::Vector6d::Constant(1.0);
static const Eigen::VectorNd    DEFAULT_KP_POSTURE      = Eigen::VectorNd::Constant(1.0);
static const Eigen::Vector6d    DEFAULT_KD_MOMENTUM     = Eigen::Vector6d::Constant(2.0);
static const Eigen::Vector6d    DEFAULT_KD_FOREARM      = Eigen::Vector6d::Constant(2.0);
static const Eigen::Vector6d    DEFAULT_KD_HAND         = Eigen::Vector6d::Constant(2.0);
static const Eigen::VectorNd    DEFAULT_KD_POSTURE      = Eigen::VectorNd::Constant(2.0);
static const double             DEFAULT_TT_MOMENTUM     = 4.0;
static const double             DEFAULT_TT_FOREARM      = 4.0;
static const double             DEFAULT_TT_HAND         = 4.0;
static const double             DEFAULT_TT_POSTURE      = 4.0;
static const int                DEFAULT_SUPPORT_PHASE       = WBR_SUPPORT_DOUBLE;
static const double             DEFAULT_NUM_DAMP            = 1e-3;
static const double             DEFAULT_INTEGRATOR_DT       = 1e-3;
static const int                DEFAULT_USE_NULLSPACE_BASE  = 0;   /// true: solver uses basis, false: it uses projectors
static const Eigen::VectorNd    DEFAULT_Q_MAX               = Eigen::VectorNd::Constant(180.0);
static const Eigen::VectorNd    DEFAULT_Q_MIN               = Eigen::VectorNd::Constant(-180.0);
static const Eigen::VectorNd    DEFAULT_DQ_MAX              = Eigen::VectorNd::Constant(DQ_MAX);
static const Eigen::VectorNd    DEFAULT_DDQ_MAX             = Eigen::VectorNd::Constant(DDQ_MAX);
static const double             DEFAULT_JNT_LIM_MIN_DIST    = 1.0;  // deg
static const double             DEFAULT_JNT_LIM_DT          = 0.1;  // sec
static const double             DEFAULT_FORCE_FRICTION      = 0.3;  // friction cone coefficient for tangential forces
static const double             DEFAULT_MOMENT_FRICTION     = 0.3;  // friction cone coefficient for normal moment
// Streaming parameters
static const Eigen::Vector3d       DEFAULT_XDES_COM        = Eigen::Vector3d::Constant(0.0);
static const Eigen::Vector7d       DEFAULT_XDES_FOREARM    = Eigen::Vector7d::Constant(0.0);
static const Eigen::Vector7d       DEFAULT_XDES_HAND       = Eigen::Vector7d::Constant(0.0);
static const Eigen::Matrix4d       DEFAULT_H_W2B           = Eigen::Matrix4d::Identity();

// *** IDs of all the module parameters
enum WholeBodyReachParamId { 
    PARAM_ID_MODULE_NAME,           PARAM_ID_CTRL_PERIOD,       PARAM_ID_ROBOT_NAME,
    
    PARAM_ID_CTRL_ALGORITHM,
    PARAM_ID_INTEGRATE_EOM,         PARAM_ID_INTEGRATOR_DT,     PARAM_ID_INTEGRATOR_DAMP,
    PARAM_ID_KP_MOMENTUM,           PARAM_ID_KP_FOREARM,
    PARAM_ID_KP_HAND,               PARAM_ID_KP_POSTURE,        PARAM_ID_KI_POSTURE,
    PARAM_ID_KP_CONSTRAINTS,
    PARAM_ID_KD_MOMENTUM,           PARAM_ID_KD_FOREARM,
    PARAM_ID_KD_HAND,               PARAM_ID_KD_POSTURE,
    PARAM_ID_TRAJ_TIME_MOMENTUM,    PARAM_ID_TRAJ_TIME_FOREARM,
    PARAM_ID_TRAJ_TIME_HAND,        PARAM_ID_TRAJ_TIME_POSTURE,
    PARAM_ID_SUPPORT_PHASE,         PARAM_ID_DYN_DAMP,          PARAM_ID_CONSTR_DAMP,
    PARAM_ID_TASK_DAMP,             PARAM_ID_USE_NULLSPACE_BASE,
    PARAM_ID_Q_MAX,                 PARAM_ID_Q_MIN,             PARAM_ID_JNT_LIM_MIN_DIST,
    PARAM_ID_DQ_MAX,                PARAM_ID_DDQ_MAX,           PARAM_ID_JNT_LIM_DT,
    PARAM_ID_FORCE_FRICTION,        PARAM_ID_MOMENT_FRICTION,   PARAM_ID_WRENCH_WEIGHTS,
    PARAM_ID_GO_DOWN_COM,           PARAM_ID_GO_DOWN_HAND,      PARAM_ID_GO_DOWN_Q,
    PARAM_ID_GO_UP_COM,             PARAM_ID_GO_UP_HAND,        PARAM_ID_GO_UP_Q,
    
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
    PARAM_ID_MOMENTUM_INTEGRAL,     PARAM_ID_JOINT_TORQUES_DES, PARAM_ID_NORMALIZED_Q,
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
new ParamProxyBasic<int>(   "ctrl alg",             PARAM_ID_CTRL_ALGORITHM,    1,                          ParamBilatBounds<int>(0, WBR_CTRL_ALG_SIZE),PARAM_IN_OUT,       &DEFAULT_CTRL_ALG,              "Id of the control algorithm MOMENTUM_SOT 0, NULLSPACE_PROJ = 1, COM_POSTURE = 2, MOMENTUM_POSTURE = 3"),
new ParamProxyBasic<int>(   "integrate EoM",        PARAM_ID_INTEGRATE_EOM,     1,                          ParamBilatBounds<int>(0, 1),                PARAM_IN_OUT,       &DEFAULT_INTEGRATE_EOM,         "If true the desired joint torques are not sent to the robot/simulator but are locally integrated"),
new ParamProxyBasic<double>("integrator dt",        PARAM_ID_INTEGRATOR_DT,     1,                          ParamBilatBounds<double>(0.0, 0.1),         PARAM_IN_OUT,       &DEFAULT_INTEGRATOR_DT,         "Timestep used by the numerical integrator"),
new ParamProxyBasic<double>("integrator damp",      PARAM_ID_INTEGRATOR_DAMP,   1,                          ParamBilatBounds<double>(0.0, 0.1),         PARAM_IN_OUT,       &DEFAULT_NUM_DAMP,              "Numerical damping used by the numerical integrator to solve the constraints"),
new ParamProxyBasic<double>("kp momentum",          PARAM_ID_KP_MOMENTUM,       6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KP_MOMENTUM.data(),     "Proportional gain for the momentum control"),
new ParamProxyBasic<double>("kp forearm",           PARAM_ID_KP_FOREARM,        6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KP_FOREARM.data(),      "Proportional gain for the forearm control"),
new ParamProxyBasic<double>("kp hand",              PARAM_ID_KP_HAND,           6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KP_HAND.data(),         "Proportional gain for the hand control"),
new ParamProxyBasic<double>("kp posture",           PARAM_ID_KP_POSTURE,        ParamSize(ICUB_DOFS,true),  ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KP_POSTURE.data(),      "Proportional gain for the joint posture control"),
new ParamProxyBasic<double>("kp constraints",       PARAM_ID_KP_CONSTRAINTS,    6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       ONE_6D.data(),                  "Proportional gain for correcting constraint drifts"),
new ParamProxyBasic<double>("kd momentum",          PARAM_ID_KD_MOMENTUM,       6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KD_MOMENTUM.data(),     "Derivative gain for the momentum control"),
new ParamProxyBasic<double>("kd forearm",           PARAM_ID_KD_FOREARM,        6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KD_FOREARM.data(),      "Derivative gain for the forearm control"),
new ParamProxyBasic<double>("kd hand",              PARAM_ID_KD_HAND,           6,                          ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KD_HAND.data(),         "Derivative gain for the hand control"),
new ParamProxyBasic<double>("kd posture",           PARAM_ID_KD_POSTURE,        ParamSize(ICUB_DOFS,true),  ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       DEFAULT_KD_POSTURE.data(),      "Derivative gain for the joint posture control"),
new ParamProxyBasic<double>("ki posture",           PARAM_ID_KI_POSTURE,        ParamSize(ICUB_DOFS,true),  ParamBilatBounds<double>(0.0, KP_MAX),      PARAM_IN_OUT,       ZERO_ND.data(),                 "Integral gain for the joint posture control"),
new ParamProxyBasic<double>("tt momentum",          PARAM_ID_TRAJ_TIME_MOMENTUM,1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_TT_MOMENTUM,           "Trajectory time for the momentum minimum jerk trajectory generator"),
new ParamProxyBasic<double>("tt forearm",           PARAM_ID_TRAJ_TIME_FOREARM, 1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_TT_FOREARM,            "Trajectory time for the forearm minimum jerk trajectory generator"),
new ParamProxyBasic<double>("tt hand",              PARAM_ID_TRAJ_TIME_HAND,    1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_TT_HAND,               "Trajectory time for the hand minimum jerk trajectory generator"),
new ParamProxyBasic<double>("tt posture",           PARAM_ID_TRAJ_TIME_POSTURE, 1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_TT_POSTURE,            "Trajectory time for the posture minimum jerk trajectory generator"),
new ParamProxyBasic<double>("dyn damp",             PARAM_ID_DYN_DAMP,          1,                          ParamBilatBounds<double>(1e-9, 1.0),        PARAM_IN_OUT,       &DEFAULT_NUM_DAMP,              "Numerical damping used to regularize the dynamics resolutions"),
new ParamProxyBasic<double>("constr damp",          PARAM_ID_CONSTR_DAMP,       1,                          ParamBilatBounds<double>(1e-9, 1.0),        PARAM_IN_OUT,       &DEFAULT_NUM_DAMP,              "Numerical damping used to regularize the constraint resolutions"),
new ParamProxyBasic<double>("task damp",            PARAM_ID_TASK_DAMP,         1,                          ParamBilatBounds<double>(1e-9, 1.0),        PARAM_IN_OUT,       &DEFAULT_NUM_DAMP,              "Numerical damping used to regularize the task resolutions"),
new ParamProxyBasic<int>(   "use nullspace base",   PARAM_ID_USE_NULLSPACE_BASE,1,                          ParamBilatBounds<int>(0, 1),                PARAM_IN_OUT,       &DEFAULT_USE_NULLSPACE_BASE,    "0: use nullspace projectors, 1: use nullspace basis"),
new ParamProxyBasic<double>("q max",                PARAM_ID_Q_MAX,             ICUB_DOFS,                  ParamConstraint<double>(),                  PARAM_IN_OUT,       DEFAULT_Q_MAX.data(),           "Joint upper bounds [deg]"),
new ParamProxyBasic<double>("q min",                PARAM_ID_Q_MIN,             ICUB_DOFS,                  ParamConstraint<double>(),                  PARAM_IN_OUT,       DEFAULT_Q_MIN.data(),           "Joint lower bounds [deg]"),
new ParamProxyBasic<double>("joint lim min dist",   PARAM_ID_JNT_LIM_MIN_DIST,  1,                          ParamLowerBound<double>(0.0),               PARAM_IN_OUT,       &DEFAULT_JNT_LIM_MIN_DIST,      "Minimum distance to maintain from the joint limits"),
new ParamProxyBasic<double>("joint lim dt",         PARAM_ID_JNT_LIM_DT,        1,                          ParamLowerBound<double>(0.001),             PARAM_IN_OUT,       &DEFAULT_JNT_LIM_DT,            "Timestep to predict future joint positions to compute joint acceleration limits"),
new ParamProxyBasic<double>("dq max",               PARAM_ID_DQ_MAX,            ICUB_DOFS,                  ParamLowerBound<double>(0.0),               PARAM_IN_OUT,       DEFAULT_DQ_MAX.data(),          "Max joint velocities [deg/s]"),
new ParamProxyBasic<double>("ddq max",              PARAM_ID_DDQ_MAX,           ICUB_DOFS,                  ParamLowerBound<double>(0.0),               PARAM_IN_OUT,       DEFAULT_DDQ_MAX.data(),         "Max joint accelerations [des/s^2]"),
new ParamProxyBasic<double>("force friction",       PARAM_ID_FORCE_FRICTION,    1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_FORCE_FRICTION,        "Friciton coefficient for tangential forces"),
new ParamProxyBasic<double>("moment friction",      PARAM_ID_MOMENT_FRICTION,   1,                          ParamLowerBound<double>(0.1),               PARAM_IN_OUT,       &DEFAULT_MOMENT_FRICTION,       "Friciton coefficient for normal moments"),
new ParamProxyBasic<double>("wrench weights",       PARAM_ID_WRENCH_WEIGHTS,    6,                          ParamBilatBounds<double>(1e-4, 1e4),        PARAM_IN_OUT,       ONE_6D.data(),                  "Weights used to penalize 6d wrenches"),
new ParamProxyBasic<double>("go down com",          PARAM_ID_GO_DOWN_COM,       3,                          ParamBilatBounds<double>(-1, 1),            PARAM_IN_OUT,       ZERO_3D.data(),                 "Desired com position associated to the go-down command"),
new ParamProxyBasic<double>("go down hand",         PARAM_ID_GO_DOWN_HAND,      3,                          ParamBilatBounds<double>(-1, 1),            PARAM_IN_OUT,       ZERO_3D.data(),                 "Desired hand position associated to the go-down command"),
new ParamProxyBasic<double>("go down q",            PARAM_ID_GO_DOWN_Q,         ICUB_DOFS,                  ParamBilatBounds<double>(-180, 180),        PARAM_IN_OUT,       ZERO_ND.data(),                 "Desired joint positions associated to the go-down command"),
new ParamProxyBasic<double>("go up com",            PARAM_ID_GO_UP_COM,         3,                          ParamBilatBounds<double>(-1, 1),            PARAM_IN_OUT,       ZERO_3D.data(),                 "Desired com position associated to the go-up command"),
new ParamProxyBasic<double>("go up hand",           PARAM_ID_GO_UP_HAND,        3,                          ParamBilatBounds<double>(-1, 1),            PARAM_IN_OUT,       ZERO_3D.data(),                 "Desired hand position associated to the go-up command"),
new ParamProxyBasic<double>("go up q",              PARAM_ID_GO_UP_Q,           ICUB_DOFS,                  ParamBilatBounds<double>(-180, 180),        PARAM_IN_OUT,       ZERO_ND.data(),                 "Desired joint positions associated to the go-up command"),
    
// ************************************************* STREAMING INPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<int>(   "support phase",        PARAM_ID_SUPPORT_PHASE,     1,                          ParamBilatBounds<int>(0, 2),                PARAM_IN_STREAM,    &DEFAULT_SUPPORT_PHASE,         "Contact support phase, 0: double, 1: triple"),
new ParamProxyBasic<double>("xd com",               PARAM_ID_XDES_COM,          3,                          ParamBilatBounds<double>(-0.26, 1.0),       PARAM_IN_STREAM,    DEFAULT_XDES_COM.data(),        "Desired 3d position of the center of mass"),
new ParamProxyBasic<double>("xd forearm",           PARAM_ID_XDES_FOREARM,      7,                          ParamBilatBounds<double>(-6.0, 6.0),        PARAM_IN_STREAM,    DEFAULT_XDES_FOREARM.data(),    "Desired position/orientation of the forearm"),
new ParamProxyBasic<double>("xd hand",              PARAM_ID_XDES_HAND,         7,                          ParamBilatBounds<double>(-6.0, 6.0),        PARAM_IN_STREAM,    DEFAULT_XDES_HAND.data(),       "Desired position/orientation of the grasping hand"),
new ParamProxyBasic<double>("qd",                   PARAM_ID_QDES,              ICUB_DOFS,                  ParamBilatBounds<double>(-200.0, 200.0),    PARAM_IN_STREAM,    ZERO_ND.data(),                 "Desired joint angles"),
new ParamProxyBasic<double>("H_w2b",                PARAM_ID_H_W2B,             16,                         ParamBilatBounds<double>(-100.0, 100.0),    PARAM_IN_STREAM,    DEFAULT_H_W2B.data(),           "Estimated rototranslation matrix between world and robot base reference frames"),
    
// ************************************************* MONITORING PARAMETERS ****************************************************************************************************************************************************************************************************************************
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
new ParamProxyBasic<double>("f inequalities rfoot", PARAM_ID_FORCE_INEQ_R_FOOT, 6,                          ParamConstraint<double>(),                  PARAM_MONITOR,      ZERO_6D.data(),                 "1-2 tang/norm force, 3 norm force, 4-5 ZMP, 6 normal moment"),
new ParamProxyBasic<double>("f inequalities lfoot", PARAM_ID_FORCE_INEQ_L_FOOT, 6,                          ParamConstraint<double>(),                  PARAM_MONITOR,      ZERO_6D.data(),                 "1-2 tang/norm force, 3 norm force, 4-5 ZMP, 6 normal moment"),
new ParamProxyBasic<double>("f inequalities forearm",PARAM_ID_FORCE_INEQ_FOREARM,3,                         ParamConstraint<double>(),                  PARAM_MONITOR,      ZERO_3D.data(),                 "1-2 tang/norm force, 3 norm force, 4-5 ZMP, 6 normal moment"),
new ParamProxyBasic<double>("momentum integral",    PARAM_ID_MOMENTUM_INTEGRAL, 6,                          ParamConstraint<double>(),                  PARAM_MONITOR,      ZERO_6D.data(),                 "Integral of the 6d centroidal momentum"),
new ParamProxyBasic<double>("taud",                 PARAM_ID_JOINT_TORQUES_DES, ICUB_DOFS,                  ParamConstraint<double>(),                  PARAM_MONITOR,      ZERO_ND.data(),                 "Desired joint torques computed by the solver"),
new ParamProxyBasic<double>("q_norm",               PARAM_ID_NORMALIZED_Q,      ICUB_DOFS,                  ParamBilatBounds<double>(-1.0, 2.0),        PARAM_MONITOR,      ZERO_ND.data(),                 "Joint angles normalized in [0 1] w.r.t. the joint limits")
};



// *** IDs of all the module command
enum WholeBodyReachCommandId { 
    COMMAND_ID_START,   COMMAND_ID_STOP,    COMMAND_ID_HELP,    COMMAND_ID_QUIT,
    COMMAND_ID_RESET_PROFILER,
    COMMAND_ID_GO_DOWN, COMMAND_ID_GO_UP,   COMMAND_ID_GRASP,
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
CommandDescription("go down",           COMMAND_ID_GO_DOWN,         "Move CoM and hand references to go down"),
CommandDescription("grasp",             COMMAND_ID_GRASP,           "Close the fingers to grasp"),
CommandDescription("go up",             COMMAND_ID_GO_UP,           "Move CoM and hand references to go back up"),
CommandDescription("help",              COMMAND_ID_HELP,            "Get instructions about how to communicate with this module"),
CommandDescription("quit",              COMMAND_ID_QUIT,            "Stop the controller and quit the module"), 
};

}

#endif
