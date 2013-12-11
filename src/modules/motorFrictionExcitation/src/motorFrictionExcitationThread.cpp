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

#include "motorFrictionIdentificationLib/motorFrictionExcitationParams.h"
#include <motorFrictionExcitation/motorFrictionExcitationThread.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/math/SVD.h>


using namespace motorFrictionExcitation;
using namespace yarp::math;
using namespace wbiIcub;

//*************************************************************************************************************************
MotorFrictionExcitationThread::MotorFrictionExcitationThread(string _name, string _robotName, int _period, ParamHelperServer *_ph, wholeBodyInterface *_wbi)
    :  RateThread(_period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi)
{
    status = EXCITATION_OFF;
    sendCmdToMotors = SEND_COMMANDS_TO_MOTORS;
    printCountdown = 0;
    excitationCounter = 0;
    _n = ICUB_DOFS;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::threadInit()
{
    ///< resize Eigen vectors
    qDeg.resize(_n);    qDeg.setZero();
    qRad.resize(_n);    qRad.setZero();
    dqJ.resize(_n);     dqJ.setZero();
    qMin.resize(_n);    qMin.setZero();
    qMax.resize(_n);    qMax.setZero();
    ftSens.resize(12);  ftSens.setZero();
    pwmDes.resize(1);   pwmDes.setZero();
    ///< resize std vectors
    freeMotionExc.resize(paramHelper->getParamProxy(PARAM_FREE_MOTION_EXCIT)->size);

    ///< link module rpc parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_FREE_MOTION_EXCIT,     freeMotionExc.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_MIN,              qMin.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_MAX,              qMax.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SEND_COMMANDS,      &sendCmdToMotors));
    ///< link module output streaming parameters to member variables
    ///< link module output monitoring parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q,                  &qDegMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_DES,            &pwmDesSingleJoint));
    
    ///< Register callbacks for some module parameters
    
    ///< Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_START,          this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_STOP,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_RESET,          this));

    ///< read robot status (to be done before initializing trajectory generators)
    if(!readRobotStatus(true))
        return false;

    ///< create and initialize trajectory generators
    
    printf("\n\n");
    return true;
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::run()
{
    paramHelper->lock();
    paramHelper->readStreamParams();

    readRobotStatus();                      // read encoders, compute positions
    if(status==EXCITATION_STARTED)
    {
        updateReferenceTrajectories();
        
        if(areDesiredMotorPwmTooLarge() || checkStopConditions())
            status = EXCITATION_FINISHED;   // stop current excitation and move to the next one
        else if( !sendMotorCommands())
            preStopOperations();
    }
    else if(status==EXCITATION_FINISHED)
    {
        preStopOperations();              // set desired PWM to 0, switch to pos ctrl
        excitationCounter++;
        if(excitationCounter >= (int)freeMotionExc.size())
            printf("Excitation process finished (%d out of %lu).\n", excitationCounter, freeMotionExc.size());
        else
        {
            printf("\nExcitation %d (out of %lu) finished.\n", excitationCounter-1, freeMotionExc.size());
            preStartOperations();
        }
    }

    paramHelper->sendStreamParams();
    paramHelper->unlock();

    printCountdown = (printCountdown>=PRINT_PERIOD) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::readRobotStatus(bool blockingRead)
{
    // read joint angles
    bool res = robot->getEstimates(ESTIMATE_JOINT_POS, qRad.data(), -1.0, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_JOINT_VEL, dqJ.data(), -1.0, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_FORCE_TORQUE, ftSens.data(), -1.0, blockingRead);
    qDeg = CTRL_RAD2DEG*qRad;
    
    //sendMsg("ft sens: "+toString(ftSens.transpose(),1), MSG_DEBUG);
    return res;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::updateReferenceTrajectories()
{
    ///< update position "error" integral
    for(unsigned int i=0; i<currentJointIds.size(); i++)
    {
        int jid = currentGlobalJointIds[i];
        posIntegral[i] += freeMotionExc[excitationCounter].ki[i]*(qDeg[jid]-freeMotionExc[excitationCounter].initialJointConfiguration[jid]);
        ///< saturate position integral
        if(posIntegral[i]>MAX_POS_INTEGRAL) 
            posIntegral[i]=MAX_POS_INTEGRAL;
        else if(posIntegral[i]<-MAX_POS_INTEGRAL) 
            posIntegral[i]=-MAX_POS_INTEGRAL;
    }

    FreeMotionExcitation *fme = &freeMotionExc[excitationCounter];
    double t = Time::now()-excitationStartTime;
    if(t<0.0)
        return false;
    ///< these operations are coefficient-wise because I'm using arrays (not matrices)
    pwmDes = pwmOffset - posIntegral + (fme->a0 + fme->a*t) * (6.28 * fme->w * t).sin();
    
    pwmDesSingleJoint = pwmDes[0];
    return true;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::checkStopConditions()
{
    for(unsigned int i=0; i<currentJointIds.size(); i++)
    {
        int jid = currentGlobalJointIds[i];
        qDegMonitor = qDeg[jid];
        double jThr = freeMotionExc[excitationCounter].jointLimitThresh[i];

        ///< check whether the joint is too close to its limit
        if(fabs(qMax[jid]-qDeg[jid])<jThr || fabs(qDeg[jid]-qMin[jid])<jThr)
        {
            printf("Joint %s got too close to its limit. Q=%.1f, Qmax=%.1f, Qmin=%.1f, Joint limit threshold=%.1f\n", 
                currentJointIds[i].description.c_str(), qDeg[jid], qMax[jid], qMin[jid], jThr);
            return true;
        }
    }
    return false;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::areDesiredMotorPwmTooLarge()
{
    for(int i=0; i<pwmDes.size(); i++)
        if(pwmDes(i)> PWM_MAX || pwmDes(i)<-PWM_MAX)
        {
            printf("Desired motor PWM are too large. Stop the excitation.\n");
            return true;
        }
    return false;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::sendMotorCommands()
{
    if(sendCmdToMotors==DO_NOT_SEND_COMMANDS_TO_MOTORS)
        return true;

    int wbiId = -1;
    for(unsigned int i=0; i<currentJointIds.size(); i++)
    {
        wbiId = robot->getJointList().localToGlobalId(currentJointIds[i]);
        assert(wbiId>=0);
        if(!robot->setControlReference(pwmDes.data()+i, wbiId))
        {
            printf("Error while setting joint %s control reference.\n", currentJointIds[i].description.c_str());
            return false;
        }
    }
    return true;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::preStartOperations()
{
    if(excitationCounter>= (int)freeMotionExc.size())
    {
        printf("Excitation process already finished.\n");
        return false;
    }
    printf("\nGoing to execute excitation %d:\n%s\n", excitationCounter, freeMotionExc[excitationCounter].toString().c_str());

    ///< no need to lock because the mutex is already locked
    if(!readRobotStatus(true))          ///< update state data
        return false;
    status = EXCITATION_STARTED;        ///< set thread status to "on"

    ///< move joints to initial configuration
    ArrayXd initialJointConf_deg = freeMotionExc[excitationCounter].initialJointConfiguration;
    moveToJointConfigurationAndWaitMotionDone(robot, initialJointConf_deg.data(), _n, 0.5);
    
    ///< Compute pwm offset
    ///< To compute a pwm offset that is not biased by the current stiction acting on the joint
    ///< I move the joint 3 degrees up and 3 degrees down, and I read the PWM value
    ///< at the two moments the joint starts moving. The average of these two values
    ///< should give me a good PWM offset
    int cjn = freeMotionExc[excitationCounter].jointId.size();  ///< current joint number
    pwmOffset.resize(cjn);
    currentJointIds.resize(cjn);
    currentGlobalJointIds.resize(cjn);
    posIntegral.resize(cjn); posIntegral.setZero();
    EstimateType estType = isRobotSimulator(robotName) ? ESTIMATE_JOINT_VEL : ESTIMATE_MOTOR_PWM;
    double pwmUp, pwmDown, q0_rad, qDes_rad, qRad_i;
    for(int i=0; i<cjn; i++)
    {
        LocalId lid = globalToLocalIcubId(freeMotionExc[excitationCounter].jointId[i]);
        currentGlobalJointIds[i] = robot->getJointList().localToGlobalId(lid);
        currentJointIds[i] = lid;
        
        q0_rad = initialJointConf_deg[currentGlobalJointIds[i]] * CTRL_DEG2RAD;
        qDes_rad = q0_rad + 3.0*CTRL_DEG2RAD;
        qRad_i = 0.0;

        ///< move joint 3 degrees up
        if(!robot->setControlReference(&qDes_rad, currentGlobalJointIds[i]))
        {
            printf("Error while moving joint %s up to estimate stiction.\n", lid.description.c_str());
            return false;
        }

        ///< as soon as joint moves read motor PWM
        do
            robot->getEstimate(ESTIMATE_JOINT_POS, lid, &qRad_i);   ///< blocking read
        while( fabs(qRad_i-q0_rad) < 0.3*CTRL_DEG2RAD);
        
        ///< read motor PWM
        //printf("Joint moved to %.1f deg.\n", qRad_i*CTRL_RAD2DEG);
        if(!robot->getEstimate(estType, lid, &pwmUp))
        {
            printf("Error while reading pwm up of joint %s. Stopping the excitation.\n", lid.description.c_str());
            return false;
        }
        
        ///< wait for motion to stop
        waitMotionDone(robot, qDes_rad*CTRL_RAD2DEG, lid, 0.2);
        
        ///< move joint 3 degrees down
        qDes_rad = q0_rad;
        robot->getEstimate(ESTIMATE_JOINT_POS, lid, &q0_rad);   ///< blocking read
        //printf("Read joint pos: %.1f\nMove joint down to %.1f.\n", q0*CTRL_RAD2DEG, qDes*CTRL_RAD2DEG);
        if(!robot->setControlReference(&qDes_rad, currentGlobalJointIds[i]))
        {
            printf("Error while moving joint %s down to estimate stiction.\n", lid.description.c_str());
            return false;
        }

        ///< wait for joint to start moving
        do
            robot->getEstimate(ESTIMATE_JOINT_POS, lid, &qRad_i);   ///< blocking read
        while( fabs(qRad_i-q0_rad) < 0.3*CTRL_DEG2RAD);

        ///< read motor PWM
        //printf("Joint moved to %.1f deg.\n", qRad_i*CTRL_RAD2DEG);
        if(!robot->getEstimate(estType, lid, &pwmDown))
        {
            printf("Error while reading pwm down of joint %s. Stopping the excitation.\n", lid.description.c_str());
            return false;
        }
        
        ///< compute pwm offset
        pwmOffset[i] = 0.5*(pwmUp+pwmDown); 
        printf("Pwm offset=%.1f. Pwm up=%.1f. Pwm down=%.1f\n", pwmOffset[i], pwmUp, pwmDown);

        ///< wait for motion to stop
        waitMotionDone(robot, qDes_rad*CTRL_RAD2DEG, lid, 0.2);
    }
    
    ///< set control mode to motor PWM
    if(sendCmdToMotors==SEND_COMMANDS_TO_MOTORS)
    {
        ControlMode ctm = isRobotSimulator(robotName) ? CTRL_MODE_VEL : CTRL_MODE_MOTOR_PWM;
        for(unsigned int i=0; i<currentJointIds.size(); i++)
        {
            if(!robot->setControlMode(ctm, pwmOffset.data()+i, currentGlobalJointIds[i]))
            {
                printf("Error while setting joint %s control mode to PWM.\n", currentJointIds[i].description.c_str());
                return false;
            }
        }
    }

    excitationStartTime = Time::now();          ///< store initial time of this excitation phase
    return true;
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::preStopOperations()
{
    // no need to lock because the mutex is already locked
    for(int i=0; i<currentGlobalJointIds.size(); i++)
    {
        pwmDes[i] = 0.0;
        if(!robot->setControlReference(pwmDes.data()+i, currentGlobalJointIds[i]))
            printf("Error while setting joint %s control reference.\n", currentJointIds[i].description.c_str());
        robot->setControlMode(CTRL_MODE_POS, 0, currentGlobalJointIds[i]);  // switch joint to position control mode
    }
    
    status = EXCITATION_OFF;                        // set thread status to "off"
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::threadRelease(){}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::parameterUpdated(const ParamProxyInterface *pd)
{
    //switch(pd->id)
    //{
    //default:
        sendMsg("A callback is registered but not managed for the parameter "+pd->name, MSG_WARNING);
    //}
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case COMMAND_ID_START:
        if(!preStartOperations())
            preStopOperations();
        break;

    case COMMAND_ID_STOP:
        preStopOperations();
        break;

    case COMMAND_ID_RESET:
        preStopOperations();
        excitationCounter = 0;
        if(!preStartOperations())
            preStopOperations();
        break;

    default:
        sendMsg("A callback is registered but not managed for the command "+cd.name, MSG_WARNING);
    }
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::sendMsg(const string &s, MsgType type)
{
    if(printCountdown==0 && type>=PRINT_MSG_LEVEL)
        printf("[MotorFrictionExcitationThread] %s\n", s.c_str());
}



//*************************************************************************************************************************
//*************************************************** UTILITY FUNCTIONS ***************************************************
//*************************************************************************************************************************
bool motorFrictionExcitation::moveToJointConfigurationAndWaitMotionDone(wholeBodyInterface *robot, 
    double *qDes_deg, const int nDoF, double tolerance_deg)
{
    double *qDes_rad = new double[nDoF];
    for(int i=0;i<nDoF;i++)
        qDes_rad[i] = qDes_deg[i] * CTRL_DEG2RAD;
    if(!robot->setControlMode(CTRL_MODE_POS, qDes_rad))
        return false;
    return waitMotionDone(robot, qDes_deg, nDoF, tolerance_deg);
}

//*************************************************************************************************************************
bool motorFrictionExcitation::waitMotionDone(iWholeBodyStates *robot, double *qDes_deg, const int nDoF, double tolerance_deg)
{
    double *qRad = new double[nDoF];
    bool motionDone = false;
    ///< wait for the joints to reach commanded configuration
    do
    {
        Time::delay(0.3);
        if(!robot->getEstimates(ESTIMATE_JOINT_POS, qRad))   ///< blocking read
            return false;
        motionDone = true;
        for(int i=0; i<nDoF; i++)
            if(fabs(CTRL_RAD2DEG*qRad[i]-qDes_deg[i])>tolerance_deg)
            {
                printf("Waiting for joint %d to stop moving, q=%.1f, qDes=%.1f\n", i, qRad[i]*CTRL_RAD2DEG, qDes_deg[i]);
                motionDone = false;
            }
    }
    while(!motionDone);
    return true;
}

//*************************************************************************************************************************
bool motorFrictionExcitation::waitMotionDone(iWholeBodyStates *robot, double qDes_deg, const LocalId &jointId, double tolerance_deg)
{
    double qRad;
    ///< wait for the joints to reach commanded configuration
    do
    {
        Time::delay(0.3);
        if(!robot->getEstimate(ESTIMATE_JOINT_POS, jointId, &qRad))   ///< blocking read
            return false;
        if(fabs(CTRL_RAD2DEG*qRad-qDes_deg)<tolerance_deg)
            return true;
    }
    while(true);
}
