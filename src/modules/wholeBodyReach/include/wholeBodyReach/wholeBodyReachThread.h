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

#ifndef WHOLE_BODY_REACH_THREAD
#define WHOLE_BODY_REACH_THREAD

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
#include <wholeBodyReach/wholeBodyReachConstants.h>
#include <wholeBodyReach/wbiStackOfTasks.h>
#include <wholeBodyReach/Logger.h>
#include <wholeBodyReach/constrainedDynamicsIntegrator.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;
using namespace std;
using namespace paramHelp;
using namespace wbi;
using namespace Eigen;

namespace wholeBodyReach
{

enum WholeBodyReachStatus { WHOLE_BODY_REACH_ON, WHOLE_BODY_REACH_OFF };

/** WholeBodyReach control thread: this thread sends torque commands to the robot motors
  * trying to track the desired position trajectory of the COM, hands and joint posture.
  * The final goal is to reach a target located on the ground in front of the robot by
  * exploiting the contact made with the other arm to balance.
  */
class WholeBodyReachThread: public RateThread, public ParamValueObserver, public CommandObserver
{
    string                  _name;              // name of the thread
    string                  _robotName;         // name of the robot
    double                  _time;              // time since the start of the thread
    
    ParamHelperServer*      _paramHelper;       // helper class for parameter management
    wholeBodyInterface*     _robot;             // interface to the robot
    wbiStackOfTasks         _solver;            // stack of task solver
    RobotState              _robotState;        // struct collecting the robot's state information
    RobotState              _robotStateIntegrator;     // used when integrating EoM
    VectorXd                _tauDes;            // desired joint torques computed by the solver
    
    struct WholeBodyReachTasks
    {
        MinJerkPDMomentumTask   momentum;
        MinJerkPDLinkPoseTask   graspHand;
        MinJerkPDLinkPoseTask   supportForearm;
        MinJerkPDPostureTask    posture;
        PlaneContactConstraint  leftFoot;
        PlaneContactConstraint  rightFoot;
        PointContactConstraint  supportForearmConstr;
        JointLimitTask          jointLimits;
        
        WholeBodyReachTasks(std::string graspHandLinkName, std::string supportForearmLinkName,
                            std::string leftFootLinkName, std::string rightFootLinkName,
                            double sampleTime, const ContactPlaneSize& footSize,
                            wbi::wholeBodyInterface* wbi)
          : momentum("momentum", sampleTime, wbi),
            graspHand("grasp hand", graspHandLinkName, sampleTime, wbi),
            supportForearm("support forearm", supportForearmLinkName, sampleTime, wbi),
            posture("posture", sampleTime, wbi),
            leftFoot("left foot", leftFootLinkName, footSize, wbi),
            rightFoot("right foot", rightFootLinkName, footSize, wbi),
            supportForearmConstr("support forearm constraint", supportForearmLinkName, wbi),
            jointLimits("joint limits", wbi)
        {}
    };
    WholeBodyReachTasks         _tasks;

    // Member variables
    WholeBodyReachStatus        _status;        // thread status ("on" when controlling, off otherwise)
    WholeBodyReachSupportPhase  _supportPhase;  // support status
    
//    int                 _printCountdown;        // every time this is 0 (i.e. every PRINT_PERIOD ms) print stuff
    int                 LINK_ID_RIGHT_FOOT;
    int                 LINK_ID_LEFT_FOOT;
    int                 _n;                     // current number of joints
    int                 _k;                     // current number of constraints
    
    JacobianMatrix      _JfootR;                // Jacobian of the right foot
    JacobianMatrix      _JfootL;                // Jacobian of the left foot
    MatrixRXd           _Jc;                    // constraint Jacobian
    JacobiSVD<MatrixRXd> _svdJcb;               // singular value decomposition of Jcb (1st 6 cols of Jc)
    VectorXd            _qjDeg;                 // joint angles (deg), variable linked to the rpc parameter "q"
    VectorXd            _dqjDeg;                // joint velocities (deg/sec), variable linked to the rpc parameter "dq"

#ifdef COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    Frame _H_base_leftFoot;                  // homogeneous transformation from robot base to left foot (i.e. world)
    Frame _Ha;                               // rotation to align foot Z axis with gravity, Ha=[0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1]
#endif
    
#ifdef DO_NOT_USE_WHOLE_BODY_STATE_INTERFACE
    // temporary replacement of state interface
    iWholeBodySensors*      _sensors;           // interface to read robot's sensors
    AWPolyEstimator*        _dqFilt;            // derivative filters for joint velocities
    VectorXd                _qJStamps;          // timestamps of encoder readings
    Vector                  _qJ_yarp;           // joint positions [rad]
    Vector                  _dqJ_yarp;          // estimated joint velocities (n)
#endif
    
    // Module parameters
    int                     _integrateEoM;      // if true it integrates the Equation of Motion
    ConstrainedDynamicsIntegrator _integrator;
    double                  _forceFriction;
    double                  _momentFriction;
    Vector6d                _kpConstraints;
    Vector6d                _wrenchWeights;     // weights used to penalize wrenches

    // Input streaming parameters
    Matrix4d            _H_w2b;                  // rototranslation from world to base reference frame

    // Output streaming parameters
    

    /************************************************* PRIVATE METHODS ******************************************************/
    void sendMsg(const string &msg, MsgType msgType=MSG_STREAM_INFO);

    /** Read the robot sensors and compute forward kinematics and Jacobians. */
    bool readRobotStatus(bool blockingRead=false);

    /** Check whether the desired joint torques are too large. */
    bool areDesiredJointTorquesTooLarge();

    /** Perform all the operations needed just before starting the controller. */
    bool preStartOperations();
    /** Perform all the operations needed just before stopping the controller. */
    void preStopOperations();

    /** Method called every time the support status changes. */
    void numberOfConstraintsChanged();

public:	
    
    /* If you define a structure having members of fixed-size vectorizable Eigen types, you must overload 
     * its "operator new" so that it generates 16-bytes-aligned pointers. Fortunately, Eigen provides you 
     * with a macro EIGEN_MAKE_ALIGNED_OPERATOR_NEW that does that for you. */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    WholeBodyReachThread(string _name, string _robotName, int _period, ParamHelperServer *_ph, wholeBodyInterface *_wbi);
	
    bool threadInit();	
    void run();
    void threadRelease();

    /** Callback function for parameter updates. */
    void parameterUpdated(const ParamProxyInterface *pd);
    /** Callback function for rpc commands. */
    void commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply);

    /** Start the controller. */
    void startController();

};

} // end namespace

#endif
