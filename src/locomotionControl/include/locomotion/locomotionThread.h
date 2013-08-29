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

#ifndef LOCOMOTION_THREAD
#define LOCOMOTION_THREAD

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <Eigen/Core>                               // import most common Eigen types

#include <wbi/wbi.h>
#include <paramHelp/paramHelpServer.h>
#include <locomotion/locomotionConstants.h>
#include <locomotion/equalityQP.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;
using namespace std;
using namespace paramHelp;
using namespace wbi;
using namespace Eigen;

namespace locomotion
{

typedef Eigen::Matrix<double,2,Dynamic,RowMajor> JacobianCom;

/** Locomotion control thread: this thread sends velocity commands to the robot motors
  * trying to track the desired position trajectory of the COM, swinging foot and joint posture.
  */
class LocomotionThread: public RateThread, public ParamObserver, public CommandObserver
{
    string              name;
    string              robotName;
    ParamHelperServer   *paramHelper;
    wholeBodyInterface  *robot;

    // Member variables
    int                 LINK_ID_RIGHT_FOOT, LINK_ID_LEFT_FOOT;
    int                 footLinkId;             // id of the controlled foot link
    int                 comLinkId;              // id of the COM
    int                 _n;                     // current number of active joints
    int                 _k;                     // current number of constraints
    Vector7d            xBase;                  // position/orientation of the floating base
    JacobianMatrix      Jcom_6xN;               // Jacobian of the center of mass (6 x N)
    JacobianCom         Jcom_2xN;               // Jacobian of the center of mass (2 x N)
    JacobianMatrix      Jfoot;                  // Jacobian of the controlled foot
    JacobianMatrix      JfootR;                 // Jacobian of the right foot
    JacobianMatrix      JfootL;                 // Jacobian of the left foot
    MatrixXd            Jc;                     // Jacobian of the constraints (k x N)

    // Module parameters
    Vector              kp_com;
    Vector              kp_foot;
    Vector              kp_posture;
    double              tt_com, tt_foot, tt_posture;    // trajectory times of min jerk trajectory generators
    VectorNi            activeJoints;                   // vector of bool indicating which joints are used (1 used, 0 blocked)
    int                 supportPhase;
    double              pinvDamp;

    // Input streaming parameters
    //Vector2d            xd_com; Vector7d            xd_foot;    VectorNd            qd;
    Vector              xd_com, xd_foot, qd;    // desired positions (use yarp vector because minJerkTrajGen wants yarp vector)
    MatrixY             H_w2b;                  // rotation matrix from world to base reference frame

    // Output streaming parameters
    Vector              xr_com, xr_foot, qr;    // reference positions (use yarp vector because minJerkTrajGen gives yarp vector)
    Vector              dxr_com, dxr_foot, dqr; // reference velocities (use yarp vector because minJerkTrajGen gives yarp vector)
    Vector              x_com, x_foot, q;       // measured positions (use yarp vector because minJerkTrajGen gives yarp vector)
    Vector              dxc_com, dxc_foot, dqc; // commanded velocities (use yarp vector because minJerkTrajGen gives yarp vector)
    Vector2d            dxc_comE;               // commanded velocity of the COM
    Vector6d            dxc_footE;
    VectorXd            dqcE;
    

    // Trajectory generators
    minJerkTrajGen      *trajGenCom, *trajGenFoot, *trajGenPosture;

    enum MsgType {MSG_DEBUG, MSG_INFO, MSG_WARNING, MSG_ERROR};
    void sendMsg(const string &msg, MsgType msgType=MSG_INFO);

    void sendMonitorData();

    bool readRobotStatus(bool blockingRead=false);
    bool updateReferenceTrajectories();
    /** Compute joint velocities by solving a hierarchy of QPs (1st QP for COM, 2nd for foot, 3rd for posture) */
    VectorXd solveTaskHierarchy();



public:	
    
    /* If you define a structure having members of fixed-size vectorizable Eigen types, you must overload 
     * its "operator new" so that it generates 16-bytes-aligned pointers. Fortunately, Eigen provides you 
     * with a macro EIGEN_MAKE_ALIGNED_OPERATOR_NEW that does that for you. */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LocomotionThread(string _name, string _robotName, int _period, ParamHelperServer *_ph, wholeBodyInterface *_wbi);
	
    bool threadInit();	
    void run();
    void threadRelease();

    /** Callback function for parameter updates. */
    void parameterUpdated(const ParamDescription &pd);
    /** Callback function for rpc commands. */
    void commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply);

};

} // end namespace

#endif
