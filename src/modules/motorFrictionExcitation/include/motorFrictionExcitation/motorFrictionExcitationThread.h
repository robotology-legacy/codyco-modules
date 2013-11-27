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
#include <paramHelp/paramHelperServer.h>
#include <motorFrictionExcitation/motorFrictionExcitationParams.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;
using namespace std;
using namespace paramHelp;
using namespace wbi;
using namespace Eigen;
using namespace motorFrictionExcitation;

namespace motorFrictionExcitation
{

enum MotorFrictionExcitationStatus 
{ 
    EXCITATION_STARTED,         // a free motion excitation has started
    EXCITATION_FINISHED,        // a free motion excitation has just finished
    EXCITATION_OFF              // controller off (either the user stopped it or all excitations have finished)
};

/** 
 * MotorFrictionExcitation control thread.
 */
class MotorFrictionExcitationThread: public RateThread, public ParamValueObserver, public CommandObserver
{
    string              name;
    string              robotName;
    ParamHelperServer   *paramHelper;
    wholeBodyInterface  *robot;

    // Member variables
    int                 printCountdown;         // every time this is 0 (i.e. every PRINT_PERIOD ms) print stuff
    int                 _n;                     // current number of active joints
    MotorFrictionExcitationStatus    status;    // thread status ("on" when controlling, off otherwise)
    int                 excitationCounter;      // counter of how many excitations have been performed
    double              excitationStartTime;    // timestamp taken at the beginning of the current excitation
    ArrayXd             pwmOffset;              // pwm to keep motor still in the starting position
    ArrayXd             pwmDes;                 // desired values of PWM for the controlled joints
    ArrayXd             dqJ;                    // joint velocities (size of vector: n)
    ArrayXd             ftSens;                 // ankle force/torque sensor readings (order is: left, right)
    vector<LocalId>     currentJointIds;        // IDs of the joints currently excited

    // Module parameters
    vector<FreeMotionExcitation>    freeMotionExc;
    ArrayXd             qMin, qMax;             // lower and upper joint bounds

    // Input streaming parameters

    // Output streaming parameters
    ArrayXd             qDeg, qRad;             // measured positions
    
    // Eigen vectors mapping Yarp vectors
    
    // Trajectory generators
    
    /************************************************* PRIVATE METHODS ******************************************************/
    void sendMsg(const string &msg, MsgType msgType=MSG_INFO);

    /** Read the robot sensors and compute forward kinematics and Jacobians. */
    bool readRobotStatus(bool blockingRead=false);

    /** Update the reference trajectories to track and compute the desired velocities for all tasks. */
    bool updateReferenceTrajectories();

    /** Return true if the desired motor PWM are too large. */
    bool areDesiredMotorPwmTooLarge();

    /** Return true if at least one of the stop conditions is verified (e.g. joint limit too close). */
    bool checkStopConditions();

    /** Send commands to the motors. Return true if the operation succeeded, false otherwise. */
    bool sendMotorCommands();

    /** Perform all the operations needed just before starting the controller. 
     * @return True iff all initialization operations went fine, false otherwise. */
    bool preStartOperations();

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
    void parameterUpdated(const ParamProxyInterface *pd);
    /** Callback function for rpc commands. */
    void commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply);

};

} // end namespace

#endif
