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
    printCountdown = 0;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::threadInit()
{
    // resize all Yarp vectors
    // resize all Eigen vectors
    // map Yarp vectors to Eigen vectors
    // link module rpc parameters to member variables
    // link module input streaming parameters to member variables
    // link module output streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q,                   qDeg.data()));      // variable size
    
    // Register callbacks for some module parameters
    
    // Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));

    // read robot status (to be done before initializing trajectory generators)
    if(!readRobotStatus(true))
        return false;

    // create and initialize trajectory generators
    
    printf("\n\n");
    return true;
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::run()
{
    paramHelper->lock();
    paramHelper->readStreamParams();

    readRobotStatus();                      // read encoders, compute positions and Jacobians
    if(status==EXCITATION_ON)
    {
        
    }

    paramHelper->sendStreamParams();
    paramHelper->unlock();

    printCountdown = (printCountdown>=PRINT_PERIOD) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::readRobotStatus(bool blockingRead)
{
    // read joint angles
    bool res = robot->getEstimates(ESTIMATE_JOINT_POS, qRad.data(), blockingRead);
    res = res && robot->getEstimates(ESTIMATE_JOINT_VEL, dqJ.data(), -1.0, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_FORCE_TORQUE, ftSens.data(), -1.0, blockingRead);
    qDeg = CTRL_RAD2DEG*qRad;
    
    //sendMsg("ft sens: "+toString(ftSens.transpose(),1), MSG_DEBUG);
    return res;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::updateReferenceTrajectories()
{

    return true;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::areDesiredJointVelTooLarge()
{
    for(int i=0; i<dqDes.size(); i++)
        if(dqDes(i)> DQ_MAX || dqDes(i)<-DQ_MAX)
            return true;
    return false;
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::preStartOperations()
{
    // no need to lock because the mutex is already locked
    readRobotStatus(true);                  // update com, foot and joint positions
    // initialize trajectory generators
    status = EXCITATION_ON;                 // set thread status to "on"
    robot->setControlMode(CTRL_MODE_MOTOR_PWM);
    /*
    for(int i=0; i<13; i++)
        robot->setControlMode(CTRL_MODE_VEL, i);   // set position control mode
    for(int i=13; i<25; i++)
        robot->setControlMode(CTRL_MODE_VEL, i);   // set position control mode
    */
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::preStopOperations()
{
    // no need to lock because the mutex is already locked
    VectorXd dqMotors = VectorXd::Zero(_n);
    robot->setControlReference(dqMotors.data());      // stop joint motors
    robot->setControlMode(CTRL_MODE_POS);   // set position control mode
    status = EXCITATION_OFF;                // set thread status to "off"
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::threadRelease()
{

}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::parameterUpdated(const ParamDescription &pd)
{
    switch(pd.id)
    {
    default:
        sendMsg("A callback is registered but not managed for the parameter "+pd.name, MSG_WARNING);
    }
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case COMMAND_ID_START:
        preStartOperations();
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
