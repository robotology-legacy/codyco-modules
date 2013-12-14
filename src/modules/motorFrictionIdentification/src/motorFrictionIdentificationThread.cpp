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

#include <motorFrictionIdentification/motorFrictionIdentificationThread.h>
#include <motorFrictionIdentificationLib/jointTorqueControlParams.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/os/Log.h>

using namespace yarp::math;
using namespace wbiIcub;
using namespace motorFrictionIdentification;

//*************************************************************************************************************************
MotorFrictionIdentificationThread::MotorFrictionIdentificationThread(string _name, string _robotName, int _period, 
    ParamHelperServer *_ph, wholeBodyInterface *_wbi, ParamHelperClient *_tc)
    :  RateThread(_period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi), torqueController(_tc)
{
    printCountdown = 0;
    _n = robot->getDoFs();
}

//*************************************************************************************************************************
bool MotorFrictionIdentificationThread::threadInit()
{
    ///< resize vectors and set them to zero
    resizeAndSetToZero(q,                       _n);
    resizeAndSetToZero(dqJ,                     _n);
    resizeAndSetToZero(dq,                      _n);
    resizeAndSetToZero(dqPos,                   _n);
    resizeAndSetToZero(dqNeg,                   _n);
    resizeAndSetToZero(extTorqueThr,            _n);
    resizeAndSetToZero(torques,                 _n);
    resizeAndSetToZero(dTorques,                _n);
    resizeAndSetToZero(gravTorques,             _n+6);
    resizeAndSetToZero(extTorques,              _n);
    resizeAndSetToZero(dqSign,                  _n);
    resizeAndSetToZero(dqSignPos,               _n);
    resizeAndSetToZero(dqSignNeg,               _n);
    resizeAndSetToZero(pwm,                     _n);
    resizeAndSetToZero(activeJoints,            _n);
    resizeAndSetToZero(currentGlobalJointIds,   _n);
    resizeAndSetToZero(zeroN,                   _n);
    stdDev.kt   =   ArrayXd::Constant(_n, 1e5);
    stdDev.kvp  =   ArrayXd::Constant(_n, 1e5);
    stdDev.kvn  =   ArrayXd::Constant(_n, 1e5);
    stdDev.kcp  =   ArrayXd::Constant(_n, 1e5);
    stdDev.kcn  =   ArrayXd::Constant(_n, 1e5);
    // @todo These arrays should have size _n rather than N_DOF 
    // I need to make the corresponding parameters in jointTorqueControl of free size
    resizeAndSetToZero(kt,                      jointTorqueControl::N_DOF);
    resizeAndSetToZero(kvp,                     jointTorqueControl::N_DOF);
    resizeAndSetToZero(kvn,                     jointTorqueControl::N_DOF);
    resizeAndSetToZero(kcp,                     jointTorqueControl::N_DOF);
    resizeAndSetToZero(kcn,                     jointTorqueControl::N_DOF);
    resizeAndSetToZero(estimateMonitor,         PARAM_NUMBER);
    resizeAndSetToZero(stdDevMonitor,           PARAM_NUMBER);
    resizeAndSetToZero(sigmaMonitor,            PARAM_NUMBER,       PARAM_NUMBER);
    resizeAndSetToZero(covarianceInv,           _n*PARAM_NUMBER,    PARAM_NUMBER);
    resizeAndSetToZero(rhs,                     _n,                 PARAM_NUMBER);

    currentJointIds.resize(_n);             ///< IDs of the joints currently excited
    inputSamples.resize(_n);
    estimators.resize(_n);

    ///< thread constants
    zero6[0] = zero6[1] = zero6[2] = zero6[3] = zero6[4] = zero6[5] = 0.0;
    ddxB[0] = ddxB[1] = ddxB[3] = ddxB[4] = ddxB[5] = 0.0;
    ddxB[2] = 9.81;     ///< gravity acceleration
    
    ///< link module rpc parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_ACTIVE_JOINTS,          activeJoints.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_IDENTIF_DELAY,          &delay));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_ZERO_JOINT_VEL_THRESH,  &zeroJointVelThr));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_ZERO_TORQUE_VEL_THRESH, &zeroTorqueVelThr));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_EXT_TORQUE_THRESH,      extTorqueThr.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_VEL_WIND_SIZE,    &jointVelEstWind));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TORQUE_VEL_WIND_SIZE,   &torqueVelEstWind));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_VEL_EST_THRESH,   &jointVelEstThr));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TORQUE_VEL_EST_THRESH,  &torqueVelEstThr));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TORQUE_FILT_CUT_FREQ,   &torqueFiltCutFreq));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_FILT_CUT_FREQ,      &pwmFiltCutFreq));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_FORGET_FACTOR,          &forgetFactor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_TO_MONITOR,       &jointMonitorName));
    ///< @todo Populate these variables and use them somehow, otherwise remove these 2 parameters
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_COVARIANCE_INV,         covarianceInv.data(),   _n*PARAM_NUMBER*PARAM_NUMBER));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_RHS,                    rhs.data(),             _n*PARAM_NUMBER));
    ///< link module output monitoring parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_VEL,              &dqMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_TORQUE,           &torqueMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_VEL_SIGN,         &signDqMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MOTOR_PWM,              &pwmMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MOTOR_PWM_PREDICT,      &pwmPredMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PARAM_ESTIMATES,        estimateMonitor.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PARAM_STD_DEV,          stdDevMonitor.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_EXT_TORQUE,             &extTorqueMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MOTOR_TORQUE_PREDICT,   &torquePredMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_IDENTIFICATION_PHASE,   &idPhaseMonitor));
    ///< link module output streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KT_STD_DEV,             stdDev.kt.data(),   _n));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KVP_STD_DEV,            stdDev.kvp.data(),  _n));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KVN_STD_DEV,            stdDev.kvn.data(),  _n));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KCP_STD_DEV,            stdDev.kcp.data(),  _n));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KCN_STD_DEV,            stdDev.kcn.data(),  _n));
    
    ///< link module jointTorqueControl parameters to member variables
    YARP_ASSERT(torqueController->linkParam(jointTorqueControl::PARAM_ID_KT,   kt.data()));
    YARP_ASSERT(torqueController->linkParam(jointTorqueControl::PARAM_ID_KVP,  kvp.data()));
    YARP_ASSERT(torqueController->linkParam(jointTorqueControl::PARAM_ID_KVN,  kvn.data()));
    YARP_ASSERT(torqueController->linkParam(jointTorqueControl::PARAM_ID_KCP,  kcp.data()));
    YARP_ASSERT(torqueController->linkParam(jointTorqueControl::PARAM_ID_KCN,  kcn.data()));
    
    ///< Register callbacks for some module parameters
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_JOINT_VEL_WIND_SIZE,    this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_TORQUE_VEL_WIND_SIZE,   this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_JOINT_VEL_EST_THRESH,   this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_TORQUE_VEL_EST_THRESH,  this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_TORQUE_FILT_CUT_FREQ,   this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_PWM_FILT_CUT_FREQ,      this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_JOINT_TO_MONITOR,       this));
    
    ///< Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_SAVE,               this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_RESET,              this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_ACTIVATE_JOINT,     this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_DEACTIVATE_JOINT,   this));

    for(int i=0; i<_n; i++)
    {
        inputSamples[i].resize(PARAM_NUMBER);
        estimators[i].setGroup1ParamSize(1);
        estimators[i].setGroup2ParamSize(PARAM_NUMBER-1);
    }
    updateJointToMonitor();
    ///< set derivative filter parameters
    robot->setEstimationParameter(ESTIMATE_MOTOR_VEL, ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE, &jointVelEstWind);
    robot->setEstimationParameter(ESTIMATE_MOTOR_VEL, ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD, &jointVelEstThr);
    robot->setEstimationParameter(ESTIMATE_MOTOR_TORQUE_DERIVATIVE, ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE, &torqueVelEstWind);
    robot->setEstimationParameter(ESTIMATE_MOTOR_TORQUE_DERIVATIVE, ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD, &torqueVelEstThr);
    robot->setEstimationParameter(ESTIMATE_MOTOR_TORQUE, ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ, &torqueFiltCutFreq);
    robot->setEstimationParameter(ESTIMATE_MOTOR_PWM,    ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ, &pwmFiltCutFreq);

    ///< read robot status
    if(!readRobotStatus(true))
        return false;
    
    printf("\n\n");
    return true;
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::run()
{
    paramHelper->lock();
    paramHelper->readStreamParams();

    readRobotStatus();
    computeInputSamples();

    for(int i=0; i<_n; i++)
    {
        if(activeJoints[i]==1)
        {
            ///< if joint is moving, estimate friction
            ///< otherwise, if there is external force, estimate motor gain
            if(fabs(dq[i])>zeroJointVelThr)
                estimators[i].feedSampleForGroup2(inputSamples[i], pwm[i]);
            else if(fabs(extTorques[i])>extTorqueThr[i] && fabs(torques[i])<TORQUE_SENSOR_SATURATION)
                estimators[i].feedSampleForGroup1(inputSamples[i], pwm[i]);
        }
    }
    
    prepareMonitorData();

    paramHelper->sendStreamParams();
    paramHelper->unlock();

    printCountdown = (printCountdown>=PRINT_PERIOD) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)
}

//*************************************************************************************************************************
bool MotorFrictionIdentificationThread::readRobotStatus(bool blockingRead)
{
    double t = Time::now() - delay;
    bool res =   robot->getEstimates(ESTIMATE_JOINT_POS,                q.data(),        t, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_JOINT_VEL,                dqJ.data(),      t, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_MOTOR_VEL,                dq.data(),       t, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_MOTOR_PWM,                pwm.data(),      t, blockingRead); 
    res = res && robot->getEstimates(ESTIMATE_MOTOR_TORQUE,             torques.data(),  t, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_MOTOR_TORQUE_DERIVATIVE,  dTorques.data(), t, blockingRead);
    res = res && robot->inverseDynamics(q.data(), Frame(), dqJ.data(), zero6, zeroN.data(), ddxB, gravTorques.data());
    extTorques = torques - gravTorques.tail(_n);
    
    dq *= CTRL_RAD2DEG;     ///< convert velocities from rad/s to deg/s

    return res;
}

//*************************************************************************************************************************
bool MotorFrictionIdentificationThread::computeInputSamples()
{
    ///< compute velocity signs
    for(int i=0; i<_n; i++)
    {
        dqPos[i]        = dq[i]>zeroJointVelThr  ?   dq[i]   :   0.0;
        dqNeg[i]        = dq[i]<-zeroJointVelThr ?   dq[i]   :   0.0;
        dqSignPos[i]    = dq[i]>zeroJointVelThr  ?   1.0     :   0.0;
        dqSignNeg[i]    = dq[i]<-zeroJointVelThr ?   -1.0    :   0.0;
        dqSign[i]       = dqSignPos[i] + dqSignNeg[i];
        
        inputSamples[i][INDEX_K_TAO]  = torques[i];
        inputSamples[i][INDEX_K_VP]   = dqPos[i];
        inputSamples[i][INDEX_K_VN]   = dqNeg[i];
        inputSamples[i][INDEX_K_CP]   = dqSignPos[i];
        inputSamples[i][INDEX_K_CN]   = dqSignNeg[i];

        ///< on the simulator generate random data samples
        if(robotName=="icubSim")
        {
            VectorXd xRand(PARAM_NUMBER);
            xRand<< 3.3, -7.2, 4.4, 8.2, 3.5;
            inputSamples[i].setRandom();
            pwm[i] = inputSamples[i].dot(xRand) + Random::normal(0, 10.0);
        }
    }

    return true;
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::prepareMonitorData()
{
    ///< ***************************** OUTPUT STREAMING VARIABLES
    for(int i=0; i<_n; i++)
    {
        estimators[i].updateParameterEstimate();
        estimators[i].getCovarianceMatrix(sigmaMonitor);
        stdDev.kt[i]    = sqrt(sigmaMonitor.diagonal()[INDEX_K_TAO]);
        stdDev.kvp[i]   = sqrt(sigmaMonitor.diagonal()[INDEX_K_VP]);
        stdDev.kvn[i]   = sqrt(sigmaMonitor.diagonal()[INDEX_K_VN]);
        stdDev.kcp[i]   = sqrt(sigmaMonitor.diagonal()[INDEX_K_CP]);
        stdDev.kcn[i]   = sqrt(sigmaMonitor.diagonal()[INDEX_K_CN]);
    }
    

    ///< ***************************** MONITOR VARIABLES
    int jid = jointMonitor;
    ///< saturate standard deviations to 1.0 to make plots nice
    stdDevMonitor[INDEX_K_TAO]  = min(stdDev.kt[jid], 1.0);
    stdDevMonitor[INDEX_K_VP]   = min(stdDev.kvp[jid], 1.0);
    stdDevMonitor[INDEX_K_VN]   = min(stdDev.kvn[jid], 1.0);
    stdDevMonitor[INDEX_K_CP]   = min(stdDev.kcp[jid], 1.0);
    stdDevMonitor[INDEX_K_CN]   = min(stdDev.kcn[jid], 1.0);
    estimators[jid].getParameterEstimate(estimateMonitor);
    dqMonitor           = dq[jid];                      ///< Velocity of the monitored joint
    torqueMonitor       = torques[jid];                 ///< Torque of the monitored joint
    signDqMonitor       = dqSign[jid];                  ///< Velocity sign of the monitored joint
    pwmMonitor          = pwm[jid];                     ///< Motor pwm of the monitored joint
    extTorqueMonitor    = extTorques[jid];              ///< External torque of the monitored joint
    ///< Prediction of current motor pwm
    estimators[jid].predictOutput(inputSamples[jid], pwmPredMonitor);   
    ///< Prediction of motor torque: tau = -(1/k_tau)(-k_tau*pwm/k_tau + k_v\dot{q} + k_c sign(\dot{q}))
    VectorXd phi = inputSamples[jid];
    double k_tau_inv = fabs(estimateMonitor[INDEX_K_TAO])>0.1 ? 1.0/estimateMonitor[INDEX_K_TAO] : 10.0;
    phi[INDEX_K_TAO] = -pwm[jid] * k_tau_inv;
    torquePredMonitor = -k_tau_inv * estimateMonitor.dot(phi);
    ///< identification phase
    idPhaseMonitor = fabs(dq[jid])>zeroJointVelThr ? 2 : (fabs(extTorques[jid])>extTorqueThr[jid] ? 1 : 0);
}

//*************************************************************************************************************************
bool MotorFrictionIdentificationThread::resetIdentification(int jid)
{
    if(jid>=_n)     ///< check if index is out of bounds
        return false;

    if(jid>=0)      ///< reset the estimator of the specified joint
    {
        estimators[jid].reset();
        return true;
    }

    ///< reset the estimators of all the joints
    for(int i=0; i<_n; i++) 
            estimators[i].reset();
    return true;
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::threadRelease(){}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::parameterUpdated(const ParamProxyInterface *pd)
{
    switch(pd->id)
    {
    case PARAM_ID_JOINT_VEL_WIND_SIZE:
        if(!robot->setEstimationParameter(ESTIMATE_MOTOR_VEL, ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE, &jointVelEstWind))
            printf("Error while setting joint velocity estimation window.");
        break;
    case PARAM_ID_JOINT_VEL_EST_THRESH:
        if(!robot->setEstimationParameter(ESTIMATE_MOTOR_VEL, ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD, &jointVelEstThr))
            printf("Error while setting joint velocity estimation threshold.");
        break;
    case PARAM_ID_TORQUE_VEL_WIND_SIZE:
        if(!robot->setEstimationParameter(ESTIMATE_MOTOR_TORQUE_DERIVATIVE, ESTIMATION_PARAM_ADAPTIVE_WINDOW_MAX_SIZE, &torqueVelEstWind))
            printf("Error while setting torque velocity estimation window.");
        break;
    case PARAM_ID_TORQUE_VEL_EST_THRESH:
        if(!robot->setEstimationParameter(ESTIMATE_MOTOR_TORQUE_DERIVATIVE, ESTIMATION_PARAM_ADAPTIVE_WINDOW_THRESHOLD, &torqueVelEstThr))
            printf("Error while setting torque velocity estimation threshold.");
        break;
    case PARAM_ID_TORQUE_FILT_CUT_FREQ:
        if(!robot->setEstimationParameter(ESTIMATE_MOTOR_TORQUE, ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ, &torqueFiltCutFreq))
            printf("Error while setting torque filter cut frequency.");
        break;
    case PARAM_ID_PWM_FILT_CUT_FREQ:
        if(!robot->setEstimationParameter(ESTIMATE_MOTOR_PWM, ESTIMATION_PARAM_LOW_PASS_FILTER_CUT_FREQ, &pwmFiltCutFreq))
            printf("Error while setting motor PWM filter cut frequency.");
        break;
    case PARAM_ID_JOINT_TO_MONITOR:
        updateJointToMonitor();
        break;
    default:
        printf("A callback is registered but not managed for the parameter %s\n",pd->name.c_str());
    }
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case COMMAND_ID_RESET:
        if(!resetIdentification(convertGlobalToLocalJointId(params)))
            reply.addString("ERROR: Reset failed.");
        break;

    case COMMAND_ID_SAVE:
        saveParametersOnFile(params, reply);
        break;

    case COMMAND_ID_ACTIVATE_JOINT:
        {
            int jid = convertGlobalToLocalJointId(params);
            if(jid>=0)
                activeJoints[jid] = 1;
            else
                reply.addString("ERROR: specified joint identifier is not valid.");
            break;
        }

    case COMMAND_ID_DEACTIVATE_JOINT:
        {
            int jid = convertGlobalToLocalJointId(params);
            if(jid>=0)
                activeJoints[jid] = 0;
            else
                reply.addString("ERROR: specified joint identifier is not valid.");
            break;
        }

    default:
        printf("A callback is registered but not managed for the command %s\n", cd.name.c_str());
    }
}

//*************************************************************************************************************************
bool MotorFrictionIdentificationThread::saveParametersOnFile(const Bottle &params, Bottle &reply)
{
    ///< read the text file name on which to save the parameters
    if(params.size()<1 || !params.get(0).isString())
    {
        reply.addString("Error, the file name is missing");
        return false;
    }
    string outputFilename = params.get(0).asString();
    
    ///< update the estimation of the parameters and the estimation state
    MatrixXd A(PARAM_NUMBER, PARAM_NUMBER);
    VectorXd b(PARAM_NUMBER);
    for(int i=0; i<_n; i++)
    {
        estimators[i].updateParameterEstimate();
        estimators[i].getParameterEstimate(estimateMonitor, sigmaMonitor);
        kt[i]   = estimateMonitor[INDEX_K_TAO];
        kvp[i]  = estimateMonitor[INDEX_K_VP];
        kvn[i]  = estimateMonitor[INDEX_K_VN];
        kcp[i]  = estimateMonitor[INDEX_K_CP];
        kcn[i]  = estimateMonitor[INDEX_K_CN];

        estimators[i].getEstimationState(A, b);
        covarianceInv.block(i*PARAM_NUMBER,0,PARAM_NUMBER,PARAM_NUMBER)  = A;
        rhs.row(i) = b;
        if(i<5) cout<<"Covariance of joint "<<i<<":\n"<<A<<endl;
    }
    
    ///< save the estimations of the parameters on text file
    int paramIds[] = { jointTorqueControl::PARAM_ID_KT, jointTorqueControl::PARAM_ID_KVP, jointTorqueControl::PARAM_ID_KVN,
        jointTorqueControl::PARAM_ID_KCP, jointTorqueControl::PARAM_ID_KCN };
    bool res = torqueController->writeParamsOnFile(outputFilename, paramIds, 5);
    
    ///< save the current values of all the module parameters on another text file (just in case)
    res = res && paramHelper->writeParamsOnFile(outputFilename+"_identificationState.ini");
    return res;
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::updateJointToMonitor()
{
    LocalId lid = globalToLocalIcubId(jointMonitorName);
    if(lid.bodyPart!=iCub::skinDynLib::BODY_PART_UNKNOWN)
        jointMonitor = robot->getJointList().localToGlobalId(lid);
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::sendMsg(const string &s, MsgType type)
{
    if(printCountdown==0 && type>=PRINT_MSG_LEVEL)
        printf("[MotorFrictionIdentificationThread] %s\n", s.c_str());
}

//*************************************************************************************************************************
int MotorFrictionIdentificationThread::convertGlobalToLocalJointId(const Bottle &b)
{
    if(b.size()==0)
        return -1;

    LocalId lid;
    if(b.get(0).isString())
    {
        string jointName = b.get(0).asString();
        lid = globalToLocalIcubId(jointName);
    }
    else if(b.get(0).isInt())
    {
        int jointId = b.get(0).asInt();
        lid = globalToLocalIcubId(jointId);
    }
    
    if(lid.bodyPart==iCub::skinDynLib::BODY_PART_UNKNOWN)
        return -1;

    int jid = robot->getJointList().localToGlobalId(lid);
    return jid;
}
