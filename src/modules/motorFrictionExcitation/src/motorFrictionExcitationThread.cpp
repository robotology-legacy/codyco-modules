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


#include <yarp/os/Time.h>
#include <yarp/os/Log.h>

#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

#include "motorFrictionIdentificationLib/motorFrictionExcitationParams.h"
#include "motorFrictionIdentificationLib/motorFrictionIdentificationParams.h"
#include <motorFrictionExcitation/motorFrictionExcitationThread.h>

using namespace motorFrictionExcitation;
using namespace yarp::math;
using namespace yarpWbi;

//*************************************************************************************************************************
MotorFrictionExcitationThread::MotorFrictionExcitationThread(string _name, string _robotName, int _period, ParamHelperServer *_ph,
    wholeBodyInterface *_wbi, ResourceFinder &rf, ParamHelperClient *_identificationModule)
    :  RateThread(_period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi), identificationModule(_identificationModule)
{
    status = EXCITATION_OFF;
    sendCmdToMotors = SEND_COMMANDS_TO_MOTORS;
    printCountdown = 0;
    freeExcCounter = 0;
    contactExcCounter = 0;
    fricStdDevThrMonitor = 0.0;
    ktStdDevThrMonitor = 0.0;
    _n = ICUB_DOFS;
    isFrictionStdDevBelowThreshold = false;

    Bottle reply;
    if(!contactExc.readFromConfigFile(rf, reply))
        printf("Error while reading contact excitation from config file: \n%s\n", reply.toString().c_str());
    printf("Results of contact excitation reading: %s\n", reply.toString().c_str());
    printf("Contact excitation value read:\n%s\n", contactExc.toString().c_str());
    reply.clear();
    if(!freeMotionExc.readFromConfigFile(rf, reply))
        printf("Error while reading free motion excitation from config file: \n%s\n", reply.toString().c_str());
    printf("Results of free motion excitation reading: %s\n", reply.toString().c_str());
    printf("Free motion excitation value read:\n%s\n", freeMotionExc.toString().c_str());
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
    activeJoints    =   ArrayXi::Constant(_n, 0);
    stdDev.kt       =   ArrayXd::Constant(_n, 1e10);
    stdDev.kvp      =   ArrayXd::Constant(_n, 1e10);
    stdDev.kvn      =   ArrayXd::Constant(_n, 1e10);
    stdDev.kcp      =   ArrayXd::Constant(_n, 1e10);
    stdDev.kcn      =   ArrayXd::Constant(_n, 1e10);
//     qMinOfCurrentJointFME = 0;
//     qMaxOfCurrentJointFME = 0;

    ///< resize std vectors

    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_MIN,              qMin.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_MAX,              qMax.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SEND_COMMANDS,      &sendCmdToMotors));
    ///< link module output streaming parameters to member variables
    ///< link module output monitoring parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q,                  &qDegMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PWM_DES,            &pwmDesSingleJoint));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KT_STD_DEV_THR,     &ktStdDevThrMonitor));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_FRIC_STD_DEV_THR,   &fricStdDevThrMonitor));

    ///< Register callbacks for some module parameters

    ///< Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_START,          this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_STOP,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_RESET,          this));

    ///< Link identification module parameters to member variables
    YARP_ASSERT(identificationModule->linkParam(motorFrictionIdentification::PARAM_ID_ACTIVE_JOINTS,    activeJoints.data(), _n));
    YARP_ASSERT(identificationModule->linkParam(motorFrictionIdentification::PARAM_ID_JOINT_TO_MONITOR, &monitoredJoint));
    YARP_ASSERT(identificationModule->linkParam(motorFrictionIdentification::PARAM_ID_KT_STD_DEV,       stdDev.kt.data(),  _n));
    YARP_ASSERT(identificationModule->linkParam(motorFrictionIdentification::PARAM_ID_KVP_STD_DEV,      stdDev.kvp.data(), _n));
    YARP_ASSERT(identificationModule->linkParam(motorFrictionIdentification::PARAM_ID_KVN_STD_DEV,      stdDev.kvn.data(), _n));
    YARP_ASSERT(identificationModule->linkParam(motorFrictionIdentification::PARAM_ID_KCP_STD_DEV,      stdDev.kcp.data(), _n));
    YARP_ASSERT(identificationModule->linkParam(motorFrictionIdentification::PARAM_ID_KCN_STD_DEV,      stdDev.kcn.data(), _n));

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

    readRobotStatus();                          ///< read encoders, compute positions
    identificationModule->readStreamParams();   ///< read std dev of parameter estimates

    if(status==EXCITATION_CONTACT)
    {
        if(checkContactStopConditions())
            status = EXCITATION_CONTACT_FINISHED;
    }
    else if(status==EXCITATION_CONTACT_FINISHED)
    {
        preStopOperations();
        contactExcCounter++;
        if(contactExcCounter >= (int)contactExc.size())
            printf("Contact excitations finished. Starting now free motion excitations.\n");
        else
            printf("Contact excitation %d finished. Moving to next contact excitation.\n", contactExcCounter-1);
        preStartOperations();
    }
    else if(status==EXCITATION_FREE_MOTION)
    {
        updateReferenceTrajectories();
        if(areDesiredMotorPwmTooLarge() || checkFreeMotionStopConditions())
            status = EXCITATION_FREE_MOTION_FINISHED;   // stop current excitation and move to the next one
        else if(!sendMotorCommands())
            preStopOperations();
    }
    else if(status==EXCITATION_FREE_MOTION_FINISHED)
    {
        preStopOperations();              ///< set desired PWM to 0, switch to pos ctrl
        printf("\nFree excitation %d (out of %lu) finished.\n", freeExcCounter, freeMotionExc.size());

        ///< if std dev is low enough then switch to next excitation, otherwise repeat this excitation
        if(isFrictionStdDevBelowThreshold)
            freeExcCounter++;
        else
            printf("Repeating the same excitation because friction parameter standard deviation is not low enough.\n");

        if(freeExcCounter < (int)freeMotionExc.size())
            preStartOperations();
        else
        {
            printf("Excitation process finished (%d out of %lu).\n", freeExcCounter, freeMotionExc.size());
            Bottle filename, reply;
            filename.addString(name.c_str());
            if(!identificationModule->sendRpcCommand(motorFrictionIdentification::COMMAND_ID_SAVE, &filename, &reply))
                printf("Error while sending 'save' command to identification module: %s\n", reply.toString().c_str());
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
    res = res && robot->getEstimates(ESTIMATE_FORCE_TORQUE_SENSOR, ftSens.data(), -1.0, blockingRead);
    qDeg = CTRL_RAD2DEG*qRad;
    return res;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::updateReferenceTrajectories()
{
    ///< update position "error" integral
    for(unsigned int i=0; i<currentJointNumericIds.size(); i++)
    {
        int jid = currentJointNumericIds[i];


        posIntegral[i] = 0;// += freeMotionExc[freeExcCounter].ki[i]*(qDeg[jid]-freeMotionExc[freeExcCounter].initialJointConfiguration[jid]);
        ///< saturate position integral
//         if(posIntegral[i]>MAX_POS_INTEGRAL)
//         {
//             posIntegral[i]=MAX_POS_INTEGRAL;
//             sendMsg(strapp("WARNING: Position integral reached its maximum value: ", MAX_POS_INTEGRAL), MSG_WARNING);
//         }
//         else if(posIntegral[i]<-MAX_POS_INTEGRAL)
//         {
//             posIntegral[i]=-MAX_POS_INTEGRAL;
//             sendMsg(strapp("WARNING: Position integral reached its minimum value: ", -MAX_POS_INTEGRAL), MSG_WARNING);
//         }
    }

    FreeMotionExcitation *fme = &freeMotionExc[freeExcCounter];
    double t = Time::now()-excitationStartTime;
    if(t<0.0)
        return false;
    ///< these operations are coefficient-wise because I'm using arrays (not matrices)
//     pwmDes = pwmOffset - posIntegral + (fme->a0 + fme->a*t) * (6.28 * fme->w * t).sin();
    pwmDes = pwmOffset + (fme->a*t) * (6.28 * fme->w * t).sin();
//     pwmDesSingleJoint = pwmDes[0];
    return true;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::checkFreeMotionStopConditions()
{
    ArrayXd maxStdDev(currentJointNumericIds.size());
    isFrictionStdDevBelowThreshold = true;
    bool areJointsTooCloseToLimits = false;
    for(unsigned int i=0; i<currentJointNumericIds.size(); i++)
    {
        int jid = currentJointNumericIds[i];
        qDegMonitor = qDeg[jid];
        double jThr = freeMotionExc[freeExcCounter].jointLimitThresh[i];

        ///< check whether the joint is too close to its limit
        if(fabs(qMax[jid]-qDeg[jid])<jThr || fabs(qDeg[jid]-qMin[jid])<jThr)
        {
            printf("Joint %s got too close to its limit. Q=%.1f, Qmax=%.1f, Qmin=%.1f, Joint limit threshold=%.1f\n",
                currentJointWbiIds[i].toString().c_str(), qDeg[jid], qMax[jid], qMin[jid], jThr);
            areJointsTooCloseToLimits = true;
        }

        ///< check whether the standard deviation of all friction parameters is under threshold
        maxStdDev[i] = max(stdDev.kvp[jid], max(stdDev.kvn[jid], max(stdDev.kcp[jid], stdDev.kcn[jid])));
        if(maxStdDev[i] > freeMotionExc[freeExcCounter].fricParamCovarThresh[i])
            isFrictionStdDevBelowThreshold = false;
        else if(currentJointNumericIds.size()==1)
            printf("Std dev of joint %s got under threshold: %f < %f\n", currentJointWbiIds[i].toString().c_str(),
                                            maxStdDev[i], freeMotionExc[freeExcCounter].fricParamCovarThresh[i]);
    }

    ///< if std dev of all joints under excitation is below threshold, stop this excitation
    if(isFrictionStdDevBelowThreshold)
        printf("Standard deviation of friction parameters of all joints got below threshold\n");

    fricStdDevThrMonitor = freeMotionExc[freeExcCounter].fricParamCovarThresh[0];

    return areJointsTooCloseToLimits || isFrictionStdDevBelowThreshold;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::checkContactStopConditions()
{
    for(unsigned int i=0; i<currentJointNumericIds.size(); i++)
    {
        if(stdDev.kt[currentJointNumericIds[i]] > contactExc[contactExcCounter].paramCovarThresh[i])
        {
            sendMsg(strapp("Standard deviation of 'kt' estimate of joint ",currentJointWbiIds[i].toString()," is too large: ",
                stdDev.kt[currentJointNumericIds[i]], " > ", contactExc[contactExcCounter].paramCovarThresh[i]), MSG_INFO);

            ktStdDevThrMonitor = contactExc[contactExcCounter].paramCovarThresh[i];

            ///< change the joint to monitor
            if(currentJointWbiIds[i].toString() != monitoredJoint)
            {
                monitoredJoint = currentJointWbiIds[i].toString();
                Bottle reply;
                if(!identificationModule->setRpcParam(motorFrictionIdentification::PARAM_ID_JOINT_TO_MONITOR, &reply))
                    printf("Error setting joint to monitor: %s\n", reply.toString().c_str());
            }
            return false;
        }
    }
    return true;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::areDesiredMotorPwmTooLarge()
{
    for(int i=0; i<pwmDes.size(); i++)
    {
        if(pwmDes(i) > qMaxOfCurrentJointFME[i])
        {
            printf("Desired position = %lf (of joint %d) is too large. Limiting it to %lf.\n", pwmDes(i) * CTRL_RAD2DEG, i, qMaxOfCurrentJointFME[i]* CTRL_RAD2DEG);
            pwmDes(i) = qMaxOfCurrentJointFME[i];
            return true;
        }
        if(pwmDes(i) < qMinOfCurrentJointFME[i])
        {
            printf("Desired position = %lf (of joint %d) is too low. Limiting it to %lf.\n", pwmDes(i) * CTRL_RAD2DEG, i, qMinOfCurrentJointFME[i]* CTRL_RAD2DEG);
            pwmDes(i) = qMinOfCurrentJointFME[i];
            return true;
        }
    }
    return false;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::sendMotorCommands()
{
    if(sendCmdToMotors==DO_NOT_SEND_COMMANDS_TO_MOTORS)
        return true;

    int wbiId = -1;
    for(unsigned int i=0; i<currentJointNumericIds.size(); i++)
    {
        wbiId = currentJointNumericIds[i];
        assert(wbiId>=0);
        if(!robot->setControlReference(pwmDes.data()+i, wbiId))
        {
            printf("Error while setting joint %s control reference.\n", currentJointWbiIds[i].toString().c_str());
            return false;
        }
    }
    return true;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::preStartOperations()
{
    if(contactExcCounter < (int)contactExc.size())
    {
        printf("\nGoing to execute contact excitation %d:\n%s\n", contactExcCounter, contactExc[contactExcCounter].toString().c_str());
        return initContactExcitation();
    }
    if(freeExcCounter < (int)freeMotionExc.size())
    {
        printf("\nGoing to execute free motion excitation %d:\n%s\n", freeExcCounter, freeMotionExc[freeExcCounter].toString().c_str());
        return initFreeMotionExcitation();
    }

    printf("Excitation process already finished.\n");
    return false;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::initContactExcitation()
{
    status = EXCITATION_CONTACT;

    ArrayXd initialJointConf_deg = contactExc[contactExcCounter].initialJointConfiguration;
    bool res = moveToJointConfigurationAndWaitMotionDone(robot, initialJointConf_deg.data(), _n, 0.5, qMax.data(), qMin.data());

    int cjn = contactExc[contactExcCounter].jointId.size();  ///< current joint number
    if(cjn==0)
        return false;
    currentJointNumericIds.resize(cjn);
    currentJointWbiIds.resize(cjn);
    Bottle jointName, reply;
    for(int i=0; i<cjn; i++)
    {
        robot->getJointList().numericIdToWbiId(contactExc[contactExcCounter].jointId[i],currentJointWbiIds[i]);
        robot->getJointList().wbiIdToNumericId(currentJointWbiIds[i],currentJointNumericIds[i]);
        YARP_ASSERT(currentJointNumericIds[i] == contactExc[contactExcCounter].jointId[i]);
        jointName.addString(currentJointWbiIds[i].toString().c_str());
        if(!identificationModule->sendRpcCommand(motorFrictionIdentification::COMMAND_ID_ACTIVATE_JOINT, &jointName, &reply))
            printf("Error activating identification of joint %s: %s\n", jointName .toString().c_str(), reply.toString().c_str());
        jointName.clear();
    }

    monitoredJoint = currentJointWbiIds[0].toString();
    if(!identificationModule->setRpcParam(motorFrictionIdentification::PARAM_ID_JOINT_TO_MONITOR, &reply))
        printf("Error setting joint to monitor: %s\n", reply.toString().c_str());

    return res;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::initFreeMotionExcitation()
{
    status = EXCITATION_FREE_MOTION;

    ///< move joints to initial configuration
    ArrayXd initialJointConf_deg = freeMotionExc[freeExcCounter].initialJointConfiguration;
    if(!moveToJointConfigurationAndWaitMotionDone(robot, initialJointConf_deg.data(), _n, 0.5, qMax.data(), qMin.data()))
        return false;

    ///< Compute pwm offset
    ///< To compute a pwm offset that is not biased by the current stiction acting on the joint
    ///< I move the joint 3 degrees up and 3 degrees down, and I read the PWM value
    ///< at the two moments the joint starts moving. The average of these two values
    ///< should give me a good PWM offset
    int cjn = freeMotionExc[freeExcCounter].jointId.size();  ///< current joint number
    pwmOffset.resize(cjn);
    currentJointNumericIds.resize(cjn);
    currentJointWbiIds.resize(cjn);
    posIntegral.resize(cjn); posIntegral.setZero();
    qMinOfCurrentJointFME.resize(cjn);
    qMaxOfCurrentJointFME.resize(cjn);
//     EstimateType estType = isRobotSimulator(robotName) ? ESTIMATE_JOINT_VEL : ESTIMATE_MOTOR_PWM;
//     double pwmUp, pwmDown, q0_rad, qDes_rad, qRad_i;
    double q0_rad = 0;
    Bottle jointName, reply;
    for(int i=0; i<cjn; i++)
    {
        ID lid;
        robot->getJointList().numericIdToWbiId(freeMotionExc[freeExcCounter].jointId[i],lid);
        robot->getJointList().wbiIdToNumericId(lid,currentJointNumericIds[i]);
        YARP_ASSERT(freeMotionExc[freeExcCounter].jointId[i] == currentJointNumericIds[i]);
        currentJointWbiIds[i] = lid;

        jointName.addString(currentJointWbiIds[i].toString().c_str());
        if(!identificationModule->sendRpcCommand(motorFrictionIdentification::COMMAND_ID_ACTIVATE_JOINT, &jointName, &reply))
            printf("Error activating identification of joint %s: %s\n", jointName .toString().c_str(), reply.toString().c_str());
        jointName.clear();

        q0_rad = initialJointConf_deg[currentJointNumericIds[i]] * CTRL_DEG2RAD;
        qMinOfCurrentJointFME[i] = qMin[currentJointNumericIds[i]] * CTRL_DEG2RAD;
        qMaxOfCurrentJointFME[i] = qMax[currentJointNumericIds[i]] * CTRL_DEG2RAD;
//         qDes_rad = q0_rad + 3.0*CTRL_DEG2RAD;
//         qRad_i = 0.0;
//
//         ///< move joint 3 degrees up
//         if(!robot->setControlReference(&qDes_rad, currentGlobalJointIds[i]))
//         {
//             printf("Error while moving joint %s up to estimate stiction.\n", lid.description.c_str());
//             return false;
//         }

        ///< as soon as joint moves read motor PWM
//         do
//             robot->getEstimate(ESTIMATE_JOINT_POS, lid, &qRad_i);   ///< blocking read
//         while( fabs(qRad_i-q0_rad) < 0.3*CTRL_DEG2RAD);

        ///< read motor PWM
        //printf("Joint moved to %.1f deg.\n", qRad_i*CTRL_RAD2DEG);
//         if(!robot->getEstimate(estType, lid, &pwmUp))
//         {
//             printf("Error while reading pwm up of joint %s. Stopping the excitation.\n", lid.description.c_str());
//             return false;
//         }

        ///< wait for motion to stop
//         waitMotionDone(robot, qDes_rad*CTRL_RAD2DEG, lid, 0.2);
//
//         ///< move joint 3 degrees down
//         qDes_rad = q0_rad;
//         robot->getEstimate(ESTIMATE_JOINT_POS, lid, &q0_rad);   ///< blocking read
//         //printf("Read joint pos: %.1f\nMove joint down to %.1f.\n", q0*CTRL_RAD2DEG, qDes*CTRL_RAD2DEG);
//         if(!robot->setControlReference(&qDes_rad, currentGlobalJointIds[i]))
//         {
//             printf("Error while moving joint %s down to estimate stiction.\n", lid.description.c_str());
//             return false;
//         }
//
//         ///< wait for joint to start moving
//         do
//             robot->getEstimate(ESTIMATE_JOINT_POS, lid, &qRad_i);   ///< blocking read
//         while( fabs(qRad_i-q0_rad) < 0.3*CTRL_DEG2RAD);
//
//         ///< read motor PWM
//         //printf("Joint moved to %.1f deg.\n", qRad_i*CTRL_RAD2DEG);
//         if(!robot->getEstimate(estType, lid, &pwmDown))
//         {
//             printf("Error while reading pwm down of joint %s. Stopping the excitation.\n", lid.description.c_str());
//             return false;
//         }

//         ///< compute pwm offset
//         pwmOffset[i] = 0.5*(pwmUp+pwmDown);
        //compute position offset
        pwmOffset[i] = q0_rad;
//         printf("Pwm offset=%.1f. Pwm up=%.1f. Pwm down=%.1f\n", pwmOffset[i], pwmUp, pwmDown);
//
//         ///< wait for motion to stop
//         waitMotionDone(robot, qDes_rad*CTRL_RAD2DEG, lid, 0.2);
    }

    monitoredJoint = currentJointWbiIds[0].toString();
    if(!identificationModule->setRpcParam(motorFrictionIdentification::PARAM_ID_JOINT_TO_MONITOR, &reply))
        printf("Error setting joint to monitor: %s\n", reply.toString().c_str());

    ///< set control mode to motor PWM
    if(sendCmdToMotors==SEND_COMMANDS_TO_MOTORS)
    {
        ControlMode ctm = wbi::CTRL_MODE_DIRECT_POSITION;// isRobotSimulator(robotName) ? CTRL_MODE_VEL : CTRL_MODE_MOTOR_PWM;
        for(unsigned int i=0; i<currentJointNumericIds.size(); i++)
        {
            if(!robot->setControlMode(ctm, pwmOffset.data()+i, currentJointNumericIds[i]))
            {
                printf("Error while setting joint %s control mode to Position Direct.\n", currentJointWbiIds[i].toString().c_str());
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
    if(status==EXCITATION_FREE_MOTION)
    {
        ArrayXd initialJointConf_deg = freeMotionExc[freeExcCounter].initialJointConfiguration;
        initialJointConf_deg *= CTRL_DEG2RAD;
        robot->setControlMode(wbi::CTRL_MODE_POS);
        robot->setControlReference(initialJointConf_deg.data());
        ///< if it was performing a free motion excitation then it stops the motors
//         for(int i=0; i<currentGlobalJointIds.size(); i++)
//         {
//             pwmDes[i] = 0.0;
//             if(!robot->setControlReference(pwmDes.data()+i, currentGlobalJointIds[i]))
//                 printf("Error while setting joint %s control reference.\n", currentJointIds[i].description.c_str());
//             robot->setControlMode(CTRL_MODE_POS, 0, currentGlobalJointIds[i]);  // switch joint to position control mode
//         }
    }

    ///< deactivate identification of joints
    Bottle jointName, reply;
    for(int i=0; i<(int)currentJointNumericIds.size(); i++)
    {
        jointName.addString(currentJointWbiIds[i].toString().c_str());
        if(!identificationModule->sendRpcCommand(motorFrictionIdentification::COMMAND_ID_DEACTIVATE_JOINT, &jointName, &reply))
            printf("Error deactivating identification of joint %s: %s\n", jointName .toString().c_str(), reply.toString().c_str());
        jointName.clear();
    }

    status = EXCITATION_OFF;                        // set thread status to "off"
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::threadRelease(){}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::parameterUpdated(const ParamProxyInterface *pd)
{
    sendMsg("A callback is registered but not managed for the parameter "+pd->name, MSG_WARNING);
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
        contactExcCounter = 0;
        freeExcCounter = 0;
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
    double *qDes_deg, const int nDoF, double tolerance_deg, double *qMaxDeg, double *qMinDeg)
{
    double *qDes_rad = new double[nDoF];
    for(int i=0;i<nDoF;i++)
    {
        qDes_rad[i] = qDes_deg[i] * CTRL_DEG2RAD;
        if(qDes_deg[i]<qMinDeg[i] || qDes_deg[i]>qMaxDeg[i])
        {
            printf("Desired joint %d position is out of bounds, qDes=%.1f, qMin=%.1f, qMax=%.1f\n",
                i, qDes_deg[i], qMinDeg[i], qMaxDeg[i]);
            return false;
        }
    }
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
bool motorFrictionExcitation::waitMotionDone(iWholeBodyStates *robot, double qDes_deg, const int &jointId, double tolerance_deg)
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
