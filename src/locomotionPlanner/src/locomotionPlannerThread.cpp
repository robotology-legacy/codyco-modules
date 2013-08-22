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

#include <locomotionPlanner\locomotionPlannerThread.h>
#include <yarp\os\Time.h>


using namespace locomotionPlanner;


LocomotionPlannerThread::LocomotionPlannerThread(string _name, string _robotName, int _period, ParamHelperServer *_ph, wholeBodyInterface *_wbi)
    :  RateThread(_period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi)
{

}

//*************************************************************************************************************************
bool LocomotionPlannerThread::threadInit()
{
    // create trajectory generators
    
    // resize vectors that are not fixed-size


    // link module rpc parameters to member variables

    // link module input streaming parameters to member variables
    assert(paramHelper->linkParam(PARAM_ID_XDES_COM,            xd_com.data()));
    assert(paramHelper->linkParam(PARAM_ID_XDES_FOOT,           xd_foot.data()));
    assert(paramHelper->linkParam(PARAM_ID_QDES,                qd.data()));
    // link module output streaming parameters to member variables
    
    // Register callbacks for some module parameters

    // Register callbacks for some module commands
    assert(paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    assert(paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));
    
    return true;
}

//*************************************************************************************************************************
void LocomotionPlannerThread::run()
{
    paramHelper->lock();
    paramHelper->readStreamParams();

    Time::delay(0.005);
    
    paramHelper->sendStreamParams();
    paramHelper->unlock();
}

//*************************************************************************************************************************
void LocomotionPlannerThread::threadRelease()
{

}

//*************************************************************************************************************************
void LocomotionPlannerThread::parameterUpdated(const ParamDescription &pd)
{
    switch(pd.id)
    {
    default:
        sendMsg("A callback is registered but not managed for the parameter "+pd.name, MSG_WARNING);
    }
}

//*************************************************************************************************************************
void LocomotionPlannerThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case COMMAND_ID_START:
        sendMsg("Starting the planner.", MSG_INFO); break;
    case COMMAND_ID_STOP:
        sendMsg("Stopping the planner.", MSG_INFO); break;
    default:
        sendMsg("A callback is registered but not managed for the command "+cd.name, MSG_WARNING);
    }
}

//*************************************************************************************************************************
void LocomotionPlannerThread::sendMsg(const string &s, MsgType type)
{
    if(type>=MSG_DEBUG)
        printf("[LocomotionPlannerThread] %s\n", s.c_str());
}