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

#ifndef _MOTOR_FRICTION_EXCITATION_THREAD
#define _MOTOR_FRICTION_EXCITATION_THREAD

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
#include <Eigen/SVD>

#include <wbi/wbi.h>
#include <paramHelp/paramHelpServer.h>
#include <motorFrictionExcitation/motorFrictionExcitationConstants.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;
using namespace std;
using namespace paramHelp;
using namespace wbi;
using namespace Eigen;

namespace motorFrictionExcitation
{

typedef Eigen::Matrix<double,2,Dynamic,RowMajor> JacobianCom;

enum MotorFrictionExcitationStatus { EXCITATION_ON, EXCITATION_OFF };

/** 
 * MotorFrictionExcitation control thread.
 */
class MotorFrictionExcitationThread: public RateThread, public ParamObserver, public CommandObserver
{
    string              name;
    string              robotName;
    ParamHelperServer   *paramHelper;
    wholeBodyInterface  *robot;

    // Member variables
    int                 printCountdown;         // every time this is 0 (i.e. every PRINT_PERIOD ms) print stuff
    int                 _n;                     // current number of active joints
    MotorFrictionExcitationStatus    status;                 // thread status ("on" when controlling, off otherwise)
    VectorXd            dq, dqJ;                // joint velocities (size of vectors: n+6, n, 6)
    //VectorXd            qMin, qMax;             // lower and upper joint bounds
    VectorXd            dqDes;
    VectorXd            ftSens;                 // ankle force/torque sensor readings (order is: left, right)

    // Module parameters

    // Input streaming parameters

    // Output streaming parameters
    Vector              qDeg, qRad;  // measured positions (use yarp vector because minJerkTrajGen gives yarp vector)
    
    // Eigen vectors mapping Yarp vectors
    
    // Trajectory generators
    
    /************************************************* PRIVATE METHODS ******************************************************/
    void sendMsg(const string &msg, MsgType msgType=MSG_INFO);

    /** Read the robot sensors and compute forward kinematics and Jacobians. */
    bool readRobotStatus(bool blockingRead=false);

    /** Update the reference trajectories to track and compute the desired velocities for all tasks. */
    bool updateReferenceTrajectories();

    /** Compute joint velocities by solving a hierarchy of QPs (1st QP for COM, 2nd for foot, 3rd for posture) */
    bool areDesiredJointVelTooLarge();

    /** Perform all the operations needed just before starting the controller. */
    void preStartOperations();
    /** Perform all the operations needed just before stopping the controller. */
    void preStopOperations();

public:	
    
    /* If you define a structure having members of fixed-size vectorizable Eigen types, you must overload 
     * its "operator new" so that it generates 16-bytes-aligned pointers. Fortunately, Eigen provides you 
     * with a macro EIGEN_MAKE_ALIGNED_OPERATOR_NEW that does that for you. */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotorFrictionExcitationThread(string _name, string _robotName, int _period, ParamHelperServer *_ph, wholeBodyInterface *_wbi);
	
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
