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
#include <paramHelp/paramHelperClient.h>

#include <motorFrictionIdentification/motorFrictionIdentificationConstants.h>
#include <motorFrictionIdentification/alternatingRecursiveLinearEstimator.h>
#include <motorFrictionIdentificationLib/motorFrictionIdentificationParams.h>

using namespace yarp::os;
using namespace std;
using namespace paramHelp;
using namespace wbi;
using namespace Eigen;
using namespace motorFrictionIdentification;


namespace motorFrictionIdentification
{

///< indeces of the estimated parameters when written in vector form (for identification)
enum MotorFrictionParamIndex
{
    INDEX_K_TAO = 0,
    INDEX_K_VP = 1,
    INDEX_K_VN = 2,
    INDEX_K_CP = 3,
    INDEX_K_CN = 4
};

/** 
 * MotorFrictionIdentification thread.
 */
class MotorFrictionIdentificationThread: public RateThread, public ParamValueObserver, public CommandObserver
{
    ///< *************** MEMBER CONSTANTS ********************
    double              zero6[6];               ///< array of 6 zeros
    ArrayXd             zeroN;                  ///< array of N zeros
    double              ddxB[6];                ///< robot base acceleration containing only gravity acceleration

    ///< *************** MEMBER VARIABLES ********************
    string              name;                   ///< name of the module instance
    string              robotName;              ///< name of the robot
    ParamHelperServer   *paramHelper;           ///< helper class for managing the module parameters
    ParamHelperClient   *torqueController;      ///< helper class for communicating with the jointTorqueControl module
    wholeBodyInterface  *robot;                 ///< interface to communicate with the robot
    vector<AlternatingRecursiveLinearEstimator>    estimators; ///< estimators, one per joint
    int                 printCountdown;         ///< every time this is 0 (i.e. every PRINT_PERIOD ms) print stuff
    int                 _n;                     ///< number of joints of the robot
    vector<LocalId>     currentJointIds;        ///< IDs of the joints currently excited
    ArrayXi             currentGlobalJointIds;  ///< global IDs of the joints currently excited
    ArrayXd             q;                      ///< joint positions
    ArrayXd             dqJ;                    ///< joint velocities
    ArrayXd             dq;                     ///< motor velocities
    ArrayXd             dqPos;                  ///< positive sample of the motor velocities
    ArrayXd             dqNeg;                  ///< negative samples of the motor velocities
    ArrayXd             torques;                ///< motor torques
    ArrayXd             dTorques;               ///< motor torque derivatives
    ArrayXd             gravTorques;            ///< torques due to gravity computed from robot's dynamic model
    ArrayXd             extTorques;             ///< torques due to external forces (measured torque - gravity torque)
    ArrayXd             dqSign;                 ///< motor velocity signes
    ArrayXd             dqSignPos;              ///< positive samples of the motor velocity signes
    ArrayXd             dqSignNeg;              ///< negative samples of the motor velocity signes
    ArrayXd             pwm;                    ///< motor PWMs
    vector<VectorXd>    inputSamples;           ///< input samples to use for identification

    ///< *************** INPUT MODULE PARAMETERS ********************
    ArrayXi     activeJoints;       ///< List of flags (0,1) indicating for which motors the identification is active
    double      delay;              ///< Delay (in sec) used before processing a sample to update the identified parameters
    double      zeroJointVelThr;    ///< Joint velocities (deg/sec) below this threshold are considered zero
    double      zeroTorqueVelThr;   ///< Torque velocities (Nm/sec) below this threshold are considered zero
    double      extTorqueThr;       ///< External torque threshold (Nm) to estimate whether there is contact
    int         jointVelEstWind;    ///< Max size of the moving window used for estimating joint velocities
    int         torqueVelEstWind;   ///< Max size of the moving window used for estimating torque velocities
    double      jointVelEstThr;     ///< Threshold used by the adaptive window estimation of joint velocity
    double      torqueVelEstThr;    ///< Threshold used by the adaptive window estimation of torque velocity
    double      torqueFiltCutFreq;  ///< Cut frequency of the low pass filter of the motor torques
    double      forgetFactor;       ///< Forgetting factor (in [0,1], 1=do not forget) used in the identification
    string      jointMonitorName;   ///< Name of the joint to monitor
    int         jointMonitor;       ///< Joint to monitor
    
    ///< *************** OUTPUT FILE PARAMETERS *************************
    MatrixXd    covarianceInv;      ///< Inverse of the covariance matrix of the parameter estimations
    ArrayXd     rhs;                ///< Right-hand side of the linear vector equation that is solved for estimating the parameters
    ArrayXd     kt;                 ///< Array of estimated parameters (motor drive gains)
    ArrayXd     kvp;                ///< Array of estimated parameters (viscous friction coefficients for positive velocities)
    ArrayXd     kvn;                ///< Array of estimated parameters (viscous friction coefficients for negative velocities)
    ArrayXd     kcp;                ///< Array of estimated parameters (Coulomb friction coefficients for positive velocities)
    ArrayXd     kcn;                ///< Array of estimated parameters (Coulomb friction coefficients for negative velocities)

    ///< *************** OUTPUT STREAMING PARAMETERS *************************
    struct
    {
        ArrayXd kt, kvp, kvn, kcp, kcn;
    } stdDev;

    ///< *************** MONITOR PARAMETERS ********************
    double      dqMonitor;          ///< Motor velocity of the monitored joint
    double      torqueMonitor;      ///< Motor torque associated to the monitored joint
    double      extTorqueMonitor;   ///< External torque at the monitored joint
    double      torquePredMonitor;  ///< Prediction of the motor torque associated to the monitored joint
    double      signDqMonitor;      ///< Velocity sign of the monitored joint
    double      pwmMonitor;         ///< Motor pwm of the monitored joint
    double      pwmPredMonitor;     ///< Prediction of the motor pwm of the monitored joint based on the current parameter estimation
    int         idPhaseMonitor;     ///< identification phase of the monitored joint (0 none, 1 torque, 2 friction)
    VectorXd    estimateMonitor;    ///< Estimates of the parameters of the monitored joint
    VectorXd    stdDevMonitor;      ///< Standard deviations of the parameters of the monitored joint
    MatrixXd    sigmaMonitor;       ///< Covariance matrix of the parameters of the monitored joint
    
    /************************************************* PRIVATE METHODS ******************************************************/
    
    /** Send out the specified message (if the specified msgType is currently allowed). */
    void sendMsg(const string &msg, MsgType msgType=MSG_INFO);

    /** Read the robot status. */
    bool readRobotStatus(bool blockingRead=false);

    /** Compute the input samples to be used for the identification of the motors and frictions. 
     * @return True iff all initialization operations went fine, false otherwise. */
    bool computeInputSamples();

    /** Prepare the data to be sent out for monitoring. */
    void prepareMonitorData();

    /** Save the current values of the identified parameters on a text file. 
     * @return True if the operation succeeded, false otherwise. */
    bool saveParametersOnFile(const Bottle &params, Bottle &reply);

    /** Reset the identification status of the specified joint. If no joint is specified
     * then it resets the identification status of all the joints.
     * @param jid Id of the joint. */
    bool resetIdentification(int jid=-1);

    /** Convert the global id contained in the specified Bottle into a local id to access
     * the vector of this thread. The Bottle b may either contain an integer id or the name
     * of the joint.
     * @param b Bottle containing the global id.
     * @return The local id, -1 if nothing was found. */
    int convertGlobalToLocalJointId(const yarp::os::Bottle &b);

    /** Update the variable jointMonitor based on the value of the variable jointMonitorName. */
    void updateJointToMonitor();

public:	
    
    /* If you define a structure having members of fixed-size vectorizable Eigen types, you must overload 
     * its "operator new" so that it generates 16-bytes-aligned pointers. Fortunately, Eigen provides you 
     * with a macro EIGEN_MAKE_ALIGNED_OPERATOR_NEW that does that for you. */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotorFrictionIdentificationThread(string _name, string _robotName, int _period, ParamHelperServer *_ph, wholeBodyInterface *_wbi, ParamHelperClient   *_tc);
	
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
