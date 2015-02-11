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

#include <Eigen/Core>                               // import most common Eigen types
#include <string>

using namespace std;

static const int       ICUB_DOFS = 23;    // number of (the main) degrees of freedom of iCub

namespace wholeBodyReach
{
    
// When COMPUTE_WORLD_2_BASE_ROTOTRANSLATION is defined the controller computes the rototranslation
// from world to floating base assuming that the left foot is always on the ground. Otherwise it 
// takes the homogeneous transformation as an input streaming parameter (i.e. H_w2b).
#define COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    
// When DO_NOT_USE_WHOLE_BODY_STATE_INTERFACE is defined the controller does not use the methods
// of the iWholeBodyState class. Instead, it uses iWholeBodySensor to read the sensor and then
// it performs some state estimation to get the desired information
#define DO_NOT_USE_WHOLE_BODY_STATE_INTERFACE
    
#define YARP_WHOLE_BODY_INTERFACE_FILE_NAME "yarpWholeBodyInterface_jtc.ini"    

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
static const Eigen::IOFormat matlabPrintFormat(Eigen::FullPrecision, Eigen::DontAlignCols, " ", ";\n", "", "", "[", "];");

/** Types of printed messages */
//enum MsgType {MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR};

// *** CONSTANTS
static const double     WBR_DEG2RAD     = 3.14/180.0;
static const double     WBR_RAD2DEG     = 180.0/3.14;
static const double     PRINT_PERIOD    = 2.0;      // period of debug prints (in sec)
//static const int        PRINT_MSG_LEVEL = MSG_DEBUG; // only messages whose type is >= PRINT_MSG_LEVEL are printed
static const double     KP_MAX          = 100.0;    // max value of proportional gains
static const double     DQ_MAX          = 50.0;     // max joint velocity allowed (deg/sec)
static const double     DDQ_MAX         = 100.0;    // max joint acceleration allowed (deg/sec^2)
static const double     TAU_MAX         = ICUB_DOFS*50.0;   // max joint torque norm allowed (Nm)
static const double     PINV_TOL        = 1e-4;     // threshold for truncated pseudoinverses
static const double     ZERO_NUM        = 1e-10;    // numerical zero (used to check if a value is zero)
static const double     FORCE_NORMAL_MIN = 20.0;    // min normal force
static const double     FORCE_NORMAL_MAX = 300.0;   // max normal force

/// true->control position, false->control position+orientation
static const bool       DEFAULT_CONTROL_POSITION_ONLY       = true;
    
/// true->control angular momentum, false->control root link's orientation
static const bool       DEFAULT_REGULATE_ANGULAR_MOMENTUM   = false;

static const string     WHOLE_BODY_DYNAMICS_NAME    = "wholeBodyDynamicsTree";
static const string     GRASP_HAND_LINK_NAME        = "r_gripper";
static const string     SUPPORT_FOREARM_LINK_NAME   = "l_forearm";
static const string     LEFT_FOOT_LINK_NAME         = "l_sole";
static const string     RIGHT_FOOT_LINK_NAME        = "r_sole";
static const ContactPlaneSize ICUB_FOOT_SIZE(0.09,0.06,0.03,0.03);

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
    
enum WholeBodyReachCtrlAlgorithm
{
    WBR_CTRL_ALG_MOMENTUM_SOT       = 0,    // stack of tasks (SoT) with Momentum control
    WBR_CTRL_ALG_NULLSPACE_PROJ     = 1,    // null-space projection (alla Righetti, Mistry)
    WBR_CTRL_ALG_COM_POSTURE        = 2,    // control CoM + posture
    WBR_CTRL_ALG_MOMENTUM_POSTURE   = 3,    // control momentum (CoM+angular momentum) + posture
    WBR_CTRL_ALG_MOMENTUM_SOT_SAFE  = 4,    // SoT with Momentum control and joint limit enforcement in force QP
    WBR_CTRL_ALG_COM_SOT            = 5,    // SoT with CoM control and joint limit enforcement in force QP
    WBR_CTRL_ALG_SIZE               = 6
};

}

#endif
