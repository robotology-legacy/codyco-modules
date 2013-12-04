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

#ifndef _MOTOR_FRICTION_IDENTIFICATION_THREAD
#define _MOTOR_FRICTION_IDENTIFICATION_THREAD

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include <yarp/os/RateThread.h>
#include <iCub/ctrl/math.h>

#include <Eigen/Core>                               // import most common Eigen types
#include <Eigen/SVD>

#include <wbi/wbi.h>
#include <paramHelp/paramHelperServer.h>

#include <motorFrictionIdentification/motorFrictionIdentificationConstants.h>
#include <motorFrictionIdentificationLib/motorFrictionIdentificationParams.h>

using namespace yarp::os;
using namespace std;
using namespace paramHelp;
using namespace wbi;
using namespace Eigen;
using namespace motorFrictionIdentification;
using namespace motorFrictionIdentificationLib;


namespace motorFrictionIdentification
{

/** 
 * MotorFrictionIdentification thread.
 */
class MotorFrictionIdentificationThread: public RateThread, public ParamValueObserver, public CommandObserver
{
    string              name;           ///< name of the module instance
    string              robotName;      ///< name of the robot
    ParamHelperServer   *paramHelper;   ///< helper class for managing the module parameters
    wholeBodyInterface  *robot;         ///< interface to communicate with the robot

    // Member variables
    int         printCountdown;         ///< every time this is 0 (i.e. every PRINT_PERIOD ms) print stuff
    int         _n;                     ///< number of joints of the robot
    vector<LocalId> currentJointIds;        ///< IDs of the joints currently excited
    ArrayXi         currentGlobalJointIds;  ///< global IDs of the joints currently excited
    ArrayXd     dq;                 ///< motor velocities
    ArrayXd     torques;            ///< motor torques
    ArrayXd     dqSign;             ///< motor velocity signes
    ArrayXd     pwm;                ///< motor PWMs

    ///< *************** INPUT MODULE PARAMETERS ********************
    
    string      outputFilename;     ///< Name of the file on which to save the state of the identification
    ArrayXd     activeJoints;       ///< List of flags (0,1) indicating for which motors the identification is active
    double      delay;              ///< Delay (in sec) used before processing a sample to update the identified parameters
    double      zeroVelThr;         ///< Velocities (deg/sec) below this threshold are considered zero
    int         velEstWind;         ///< Max size of the moving window used for estimating joint velocities
    double      forgetFactor;       ///<Forgetting factor (in [0,1], 1=do not forget) used in the identification
    int         jointMonitor;       ///<Joint to monitor
    
    ///< *************** OUTPUT FILE PARAMETERS *************************
    MatrixXd    covarianceInv;      ///< Inverse of the covariance matrix of the parameter estimations
    ArrayXd     rhs;                ///< Right-hand side of the linear vector equation that is solved for estimating the parameters

    ///< *************** MONITOR PARAMETERS ********************
    double      dqMonitor;          ///< Velocity of the monitored joint
    double      torqueMonitor;      ///< Torque of the monitored joint
    double      signDqMonitor;      ///< Velocity sign of the monitored joint
    double      pwmMonitor;         ///< Motor pwm of the monitored joint
    VectorPd    estimateMonitor;    ///< Estimates of the parameters of the monitored joint
    VectorPd    variancesMonitor;   ///< Variances of the parameters of the monitored joint
    
    /************************************************* PRIVATE METHODS ******************************************************/
    
    /** Send out the specified message (if the specified msgType is currently allowed). */
    void sendMsg(const string &msg, MsgType msgType=MSG_INFO);

    /** Read the robot status. */
    bool readRobotStatus(bool blockingRead=false);

    /** Perform all the operations needed just before starting the identification of a joint. 
     * @return True iff all initialization operations went fine, false otherwise. */
    bool preStartOperations();

    /** Perform all the operations needed just before stopping the identification of a joint. */
    void preStopOperations();

public:	
    
    /* If you define a structure having members of fixed-size vectorizable Eigen types, you must overload 
     * its "operator new" so that it generates 16-bytes-aligned pointers. Fortunately, Eigen provides you 
     * with a macro EIGEN_MAKE_ALIGNED_OPERATOR_NEW that does that for you. */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotorFrictionIdentificationThread(string _name, string _robotName, int _period, ParamHelperServer *_ph, wholeBodyInterface *_wbi);
	
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
