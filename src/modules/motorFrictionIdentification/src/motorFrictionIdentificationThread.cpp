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
    dq.resize(_n);     
    dq.setZero();
    torques.resize(_n);
    torques.setZero();
    dqSign.resize(_n);
    dqSign.setZero();
    pwm.resize(_n);
    pwm.setZero();
    currentJointIds.resize(_n);             ///< IDs of the joints currently excited
    currentGlobalJointIds.resize(_n);       ///< global IDs of the joints currently excited
    activeJoints.resize(_n);                ///< List of flags (0,1) indicating for which motors the identification is active
    activeJoints.setZero();
    covarianceInv.resize(_n,PARAM_NUMBER*PARAM_NUMBER); ///< Inverse of the covariance matrix of the parameter estimations
    covarianceInv.setZero();
    rhs.resize(_n*PARAM_NUMBER);            ///< Right-hand side of the linear vector equation that is solved for estimating the parameters
    rhs.setZero();
    
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
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PARAM_ESTIMATES,    &estimateMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PARAM_VARIANCE,     &variancesMonitor));
    
    ///< Register callbacks for some module parameters
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_ACTIVE_JOINTS,      this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_VEL_EST_WIND_SIZE,  this));
    
    ///< Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_SAVE,           this));

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

    ///< convert velocities from rad/s to deg/s
    dq *= CTRL_RAD2DEG;

    ///< compute velocity signes
    for(int i=0; i<_n; i++)
    {
        if(dq[i]>zeroVelThr)
            dqSign[i] = 1.0;
        else if(dq[i]<-zeroVelThr)
            dqSign[i] = -1.0;
        else
            dqSign[i] = 0.0;
    }

    ///< monitor variables
    int jid = robot->getJointList().localToGlobalId(globalToLocalIcubId(jointMonitor));
    dqMonitor       = dq[jid];          ///< Velocity of the monitored joint
    torqueMonitor   = torques[jid];     ///< Torque of the monitored joint
    signDqMonitor   = dqSign[jid];      ///< Velocity sign of the monitored joint
    pwmMonitor      = pwm[jid];         ///< Motor pwm of the monitored joint
    //estimateMonitor = ??;    ///< Estimates of the parameters of the monitored joint
    //variancesMonitor = ;   ///< Variances of the parameters of the monitored joint
    
    return res;
}

//*************************************************************************************************************************
bool MotorFrictionIdentificationThread::preStartOperations()
{
    ///< no need to lock because the mutex is already locked
    if(!readRobotStatus(true))          ///< update state data
        return false;
    
    return true;
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::preStopOperations()
{
    
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
