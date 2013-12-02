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
    status = IDENTIFICATION_OFF;
    printCountdown = 0;
    _n = ICUB_DOFS;
}

//*************************************************************************************************************************
bool MotorFrictionIdentificationThread::threadInit()
{
    ///< resize Eigen vectors
    dqJ.resize(_n);     dqJ.setZero();
    ///< resize std vectors
    
    ///< link module rpc parameters to member variables
    ///< link module output streaming parameters to member variables
    ///< link module output monitoring parameters to member variables
    
    ///< Register callbacks for some module parameters
    
    ///< Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));

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

    readRobotStatus();                      // read encoders, compute positions
    if(status==IDENTIFICATION_STARTED)
    {
        
    }
    else if(status==IDENTIFICATION_FINISHED)
    {
        
    }

    paramHelper->sendStreamParams();
    paramHelper->unlock();

    printCountdown = (printCountdown>=PRINT_PERIOD) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)
}

//*************************************************************************************************************************
bool MotorFrictionIdentificationThread::readRobotStatus(bool blockingRead)
{
    // estimate joint velocities
    bool res = robot->getEstimates(ESTIMATE_JOINT_VEL, dqJ.data(), -1.0, blockingRead);
    
    return res;
}

//*************************************************************************************************************************
bool MotorFrictionIdentificationThread::preStartOperations()
{
    ///< no need to lock because the mutex is already locked
    if(!readRobotStatus(true))          ///< update state data
        return false;
    status = IDENTIFICATION_STARTED;        ///< set thread status to "on"

    int cjn = 1; ///< current joint number
    currentJointIds.resize(cjn);
    currentGlobalJointIds.resize(cjn);
    for(int i=0; i<cjn; i++)
    {
        /*LocalId lid = globalToLocalIcubId(freeMotionExc[excitationCounter].jointId[i]);
        currentGlobalJointIds[i] = robot->getJointList().localToGlobalId(lid);
        currentJointIds[i] = lid;*/
    }
    
    return true;
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::preStopOperations()
{
    status = IDENTIFICATION_OFF;                        // set thread status to "off"
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::threadRelease(){}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::parameterUpdated(const ParamProxyInterface *pd)
{
    //switch(pd->id)
    //{
    //default:
        sendMsg("A callback is registered but not managed for the parameter "+pd->name, MSG_WARNING);
    //}
}

//*************************************************************************************************************************
void MotorFrictionIdentificationThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
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
void MotorFrictionIdentificationThread::sendMsg(const string &s, MsgType type)
{
    if(printCountdown==0 && type>=PRINT_MSG_LEVEL)
        printf("[MotorFrictionIdentificationThread] %s\n", s.c_str());
}
