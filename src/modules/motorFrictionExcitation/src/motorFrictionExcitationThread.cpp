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

#include "motorFrictionExcitation/motorFrictionExcitationParams.h"
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
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));

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
        excitationCounter++;
        if(excitationCounter >= (int)freeMotionExc.size())
        {
            printf("Excitation process finished (%d out of %d).\n", excitationCounter, freeMotionExc.size());
            status = EXCITATION_OFF;
        }
        else
        {
            printf("\nExcitation %d (out of %d) finished.\n", excitationCounter-1, freeMotionExc.size());
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
    FreeMotionExcitation *fme = &freeMotionExc[excitationCounter];
    double t = Time::now()-excitationStartTime;
    if(t<0.0)
        return false;
    ///< these operations are coefficient-wise because I'm using arrays (not matrices)
    pwmDes = pwmOffset + (fme->a0 + fme->a*t) * (6.28 * fme->w * t).sin();
    pwmDesSingleJoint = pwmDes[0];
    return true;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::checkStopConditions()
{
    for(unsigned int i=0; i<currentJointIds.size(); i++)
    {
        int jid = robot->getJointList().localToGlobalId(currentJointIds[i]);
        qDegMonitor = qDeg[jid];
        double jThr = freeMotionExc[excitationCounter].jointLimitThresh[i];
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
    robot->setControlMode(CTRL_MODE_POS);
    ArrayXd initialJointConfRad = CTRL_DEG2RAD * freeMotionExc[excitationCounter].initialJointConfiguration;
    robot->setControlReference(initialJointConfRad.data());
    
    // @todo Improve this by checking whether the joints have actually reached the desired configuration
    //       or checking whether the joint velocity is zero
    Time::delay(3.0);                   ///< wait for the joints to reach commanded configuration
    
    ///< Compute pwm offset
    ///< To compute a pwm offset that is not biased by the current stiction acting on the joint
    ///< I move the joint 2 degrees up and 2 degrees down, and I read the PWM value
    ///< at the two moments the joint starts moving. The average of these two values
    ///< should give me a good PWM offset
    int cjn = freeMotionExc[excitationCounter].jointId.size();  ///< current joint number
    pwmOffset.resize(cjn);
    currentJointIds.resize(cjn);
    currentGlobalJointIds.resize(cjn);
    EstimateType estType = isRobotSimulator(robotName) ? ESTIMATE_JOINT_VEL : ESTIMATE_MOTOR_PWM;
    double pwmUp, pwmDown;
    for(int i=0; i<cjn; i++)
    {
        LocalId lid = globalToLocalIcubId(freeMotionExc[excitationCounter].jointId[i]);
        currentGlobalJointIds[i] = robot->getJointList().localToGlobalId(lid);
        currentJointIds[i] = lid;
        
        double q0 = initialJointConfRad[currentGlobalJointIds[i]];
        double qDes = q0 + 2.0*CTRL_DEG2RAD;
        double qRad_i = 0.0;
        /*printf("Configuration joint id: %d, wbiLocalId=%s, wbiGlobalId=%d\n", freeMotionExc[excitationCounter].jointId[i], 
            lid.description.c_str(), currentGlobalJointIds[i]);*/
        printf("q0 = %.1f, qDes=%.1f\n", q0*CTRL_RAD2DEG, qDes*CTRL_RAD2DEG);

        ///< move joint 2 degrees up
        if(!robot->setControlReference(&qDes, currentGlobalJointIds[i]))
        {
            printf("Error while moving joint %s up to estimate stiction.\n", lid.description.c_str());
            return false;
        }

        ///< as soon as joint moves read motor PWM
        do
        {
            robot->getEstimate(ESTIMATE_JOINT_POS, lid, &qRad_i);   ///< blocking read
            //robot->getEstimates(ESTIMATE_JOINT_POS, qRad.data());   ///< blocking read
            //qRad_i = qRad[currentGlobalJointIds[i]];
        }
        while( fabs(qRad_i-q0) < 0.5*CTRL_DEG2RAD);
        
        ///< read motor PWM
        printf("Joint moved to %.1f deg.\n", qRad_i*CTRL_RAD2DEG);
        if(!robot->getEstimate(estType, lid, &pwmUp))
        {
            printf("Error while reading pwm up of joint %s. Stopping the excitation.\n", lid.description.c_str());
            return false;
        }
        Time::delay(2.0);   ///< wait for motion to stop
        
        ///< move joint 2 degrees down
        qDes = q0;
        robot->getEstimate(ESTIMATE_JOINT_POS, lid, &q0);   ///< blocking read
        printf("Read joint pos: %.1f\nMove joint down to %.1f.\n", q0*CTRL_RAD2DEG, qDes*CTRL_RAD2DEG);
        if(!robot->setControlReference(&qDes, currentGlobalJointIds[i]))
        {
            printf("Error while moving joint %s down to estimate stiction.\n", lid.description.c_str());
            return false;
        }

        ///< wait for joint to start moving
        printf("Wait for joint to start moving\n");
        do
        {
            robot->getEstimate(ESTIMATE_JOINT_POS, lid, &qRad_i);   ///< blocking read
            /*robot->getEstimates(ESTIMATE_JOINT_POS, qRad.data());   ///< blocking read
            qRad_i = qRad[currentGlobalJointIds[i]];*/
        }
        while( fabs(qRad_i-q0) < 0.5*CTRL_DEG2RAD);

        ///< read motor PWM
        printf("Joint moved to %.1f deg.\n", qRad_i*CTRL_RAD2DEG);
        if(!robot->getEstimate(estType, lid, &pwmDown))
        {
            printf("Error while reading pwm down of joint %s. Stopping the excitation.\n", lid.description.c_str());
            return false;
        }

        pwmOffset[i] = 0.5*(pwmUp+pwmDown); 
        printf("Pwm offset=%.1f. Pwm up=%.1f. Pwm down=%.1f\n", pwmOffset[i], pwmUp, pwmDown);
    }
    
    ///< set control mode to motor PWM
    if(sendCmdToMotors==SEND_COMMANDS_TO_MOTORS)
    {
        int wbiId = -1;
        ControlMode ctm = isRobotSimulator(robotName) ? CTRL_MODE_VEL : CTRL_MODE_MOTOR_PWM;
        for(unsigned int i=0; i<currentJointIds.size(); i++)
        {
            wbiId = robot->getJointList().localToGlobalId(currentJointIds[i]);
            if(!robot->setControlMode(ctm, pwmOffset.data()+i, wbiId))
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
    robot->setControlMode(CTRL_MODE_POS);           // set position control mode
    status = EXCITATION_OFF;                        // set thread status to "off"
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::threadRelease()
{

}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::parameterUpdated(const ParamProxyInterface *pd)
{
    switch(pd->id)
    {
    default:
        sendMsg("A callback is registered but not managed for the parameter "+pd->name, MSG_WARNING);
    }
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
