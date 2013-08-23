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
#include <locomotion\locomotionConstants.h>
#include <yarp\os\Time.h>


using namespace locomotion;
using namespace locomotionPlanner;


LocomotionPlannerThread::LocomotionPlannerThread(string _name, string _robotName, ParamHelperServer *_ph, ParamHelperClient *_lc, wholeBodyInterface *_wbi)
    :  name(_name), robotName(_robotName), paramHelper(_ph), locoCtrl(_lc), robot(_wbi)
{
    mustStop = false;
}

//*************************************************************************************************************************
bool LocomotionPlannerThread::threadInit()
{
    // resize vectors that are not fixed-size

    // link module rpc parameters to member variables

    // link controller input streaming parameters to member variables
    assert(locoCtrl->linkParam(PARAM_ID_XDES_COM,            xd_com.data()));
    assert(locoCtrl->linkParam(PARAM_ID_XDES_FOOT,           xd_foot.data()));
    assert(locoCtrl->linkParam(PARAM_ID_QDES,                qd.data()));
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
    while(!mustStop)
    {
        locoCtrl->readStreamParams();

        xd_com.setRandom();
        xd_foot.setRandom();
        qd.setRandom();
        Time::delay(2.0);
    
        locoCtrl->sendStreamParams();
    }
}

//*************************************************************************************************************************
void LocomotionPlannerThread::threadRelease()
{

}

//*************************************************************************************************************************
void LocomotionPlannerThread::parameterUpdated(const ParamDescription &pd)
{
    //switch(pd.id)
    //{
    //default:
    sendMsg("A callback is registered but not managed for the parameter "+pd.name, MSG_WARNING);
    //}
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