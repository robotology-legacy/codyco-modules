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
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/os/Log.h>
#include <yarp/math/SVD.h>


using namespace yarp::math;
using namespace wbiIcub;
using namespace motorFrictionIdentification;
using namespace motorFrictionIdentificationLib;

//*************************************************************************************************************************
MotorFrictionIdentificationThread::MotorFrictionIdentificationThread(string _name, string _robotName, int _period, 
    ParamHelperServer *_ph, wholeBodyInterface *_wbi)
    :  RateThread(_period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi)
{
    printCountdown = 0;
    _n = robot->getDoFs();
}

//*************************************************************************************************************************
bool MotorFrictionIdentificationThread::threadInit()
{
    ///< resize vectors and set them to zero
    resizeAndSetToZero(dq, _n);
    resizeAndSetToZero(dqPos, _n);
    resizeAndSetToZero(dqNeg, _n);
    resizeAndSetToZero(torques, _n);
    resizeAndSetToZero(dqSign, _n);
    resizeAndSetToZero(dqSignPos, _n);
    resizeAndSetToZero(dqSignNeg, _n);
    resizeAndSetToZero(pwm, _n);
    resizeAndSetToZero(activeJoints, _n);
    resizeAndSetToZero(currentGlobalJointIds, _n);
    resizeAndSetToZero(rhs, _n*PARAM_NUMBER);
    resizeAndSetToZero(estimateMonitor, PARAM_NUMBER);
    resizeAndSetToZero(variancesMonitor, PARAM_NUMBER);
    resizeAndSetToZero(sigmaMonitor, PARAM_NUMBER, PARAM_NUMBER);
    resizeAndSetToZero(covarianceInv, _n, PARAM_NUMBER*PARAM_NUMBER);

    currentJointIds.resize(_n);             ///< IDs of the joints currently excited
    inputSamples.resize(_n);
    estimators.resize(_n);
    
    
    ///< link module rpc parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_OUTPUT_FILENAME,    &outputFilename));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_ACTIVE_JOINTS,      activeJoints.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_IDENTIF_DELAY,      &delay));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_ZERO_VEL_THRESH,    &zeroVelThr));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_VEL_EST_WIND_SIZE,  &velEstWind));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_FORGET_FACTOR,      &forgetFactor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_TO_MONITOR,   &jointMonitor));

    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_COVARIANCE_INV,     covarianceInv.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_RHS,                rhs.data()));
    ///< link module output monitoring parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_VEL,          &dqMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_TORQUE,       &torqueMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JOINT_VEL_SIGN,     &signDqMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MOTOR_PWM,          &pwmMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MOTOR_PWM_PREDICT,  &pwmPredMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PARAM_ESTIMATES,    estimateMonitor.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PARAM_VARIANCE,     variancesMonitor.data()));
    
    ///< Register callbacks for some module parameters
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_ACTIVE_JOINTS,      this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_VEL_EST_WIND_SIZE,  this));
    
    ///< Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_SAVE,           this));

    for(int i=0; i<_n; i++)
    {
        inputSamples[i].resize(PARAM_NUMBER);
        estimators[i].setParamSize(PARAM_NUMBER);
    }

    ///< read robot status
    if(!readRobotStatus(true))
        return false;
    
    // don't know if this stuff is useful
    /*for(int i=0; i<_n; i++)
    {
        LocalId lid = globalToLocalIcubId(freeMotionExc[excitationCounter].jointId[i]);
        currentGlobalJointIds[i] = robot->getJointList().localToGlobalId(lid);
        currentJointIds[i] = lid;
    }*/
    
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
            estimators[i].feedSample(inputSamples[i], pwm[i]);
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
    bool res =   robot->getEstimates(ESTIMATE_MOTOR_VEL,    dq.data(),      t, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_MOTOR_PWM,    pwm.data(),     t, blockingRead); 
    res = res && robot->getEstimates(ESTIMATE_MOTOR_TORQUE, torques.data(), t, blockingRead);

    dq *= CTRL_RAD2DEG;     ///< convert velocities from rad/s to deg/s

    return res;
}

//*************************************************************************************************************************
bool MotorFrictionIdentificationThread::computeInputSamples()
{
    ///< compute velocity signs
    for(int i=0; i<_n; i++)
    {
        if(dq[i]>zeroVelThr)
        {
            dqPos[i]        = dq[i];
            dqNeg[i]        = 0.0;
            dqSign[i]       = 1.0;
            dqSignPos[i]    = 1.0;
            dqSignNeg[i]    = 0.0;
        }
        else if(dq[i]<-zeroVelThr)
        {
            dqPos[i]        = 0.0;
            dqNeg[i]        = dq[i];
            dqSign[i]       = -1.0;
            dqSignPos[i]    = 0.0;
            dqSignNeg[i]    = 1.0;
        }
        else
        {
            dqPos[i]        = 0.0;
            dqNeg[i]        = 0.0;
            dqSign[i]       = 0.0;
            dqSignPos[i]    = 0.0;
            dqSignNeg[i]    = 0.0;
        }

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
    ///< monitor variables
    int jid = robot->getJointList().localToGlobalId(globalToLocalIcubId(jointMonitor));
    estimators[jid].updateParameterEstimation();    ///< Estimates of the parameters of the monitored joint
    estimators[jid].getCurrentParameterEstimate(estimateMonitor, sigmaMonitor);
    variancesMonitor = sigmaMonitor.diagonal();     ///< Variances of the parameters of the monitored joint
    dqMonitor       = dq[jid];                      ///< Velocity of the monitored joint
    torqueMonitor   = torques[jid];                 ///< Torque of the monitored joint
    signDqMonitor   = dqSign[jid];                  ///< Velocity sign of the monitored joint
    pwmMonitor      = pwm[jid];                     ///< Motor pwm of the monitored joint
    estimators[jid].predictOutput(inputSamples[jid], pwmPredMonitor);   ///< Prediction of current motor pwm
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::threadRelease(){}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::parameterUpdated(const ParamProxyInterface *pd)
{
    switch(pd->id)
    {
    case PARAM_ID_ACTIVE_JOINTS:
        printf("Param active joints changed\n");
        break;
    case PARAM_ID_VEL_EST_WIND_SIZE:
        printf("Param velocity estimation window size changed\n");
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
    case COMMAND_ID_SAVE:
        printf("Save command received.\n");
        break;
    default:
        printf("A callback is registered but not managed for the command %s\n", cd.name.c_str());
    }
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::sendMsg(const string &s, MsgType type)
{
    if(printCountdown==0 && type>=PRINT_MSG_LEVEL)
        printf("[MotorFrictionIdentificationThread] %s\n", s.c_str());
}
