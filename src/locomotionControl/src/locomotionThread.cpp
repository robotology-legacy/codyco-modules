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

#include <locomotion\locomotionThread.h>
#include <yarp\os\Time.h>


using namespace locomotion;


LocomotionThread::LocomotionThread(string _name, string _robotName, int _period, ParamHelperServer *_ph, wholeBodyInterface *_wbi)
    :  RateThread(_period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi)
{

}

//*************************************************************************************************************************
bool LocomotionThread::threadInit()
{
    // create trajectory generators
    trajGenCom      = new minJerkTrajGen(DEFAULT_KP_COM.rows(),     getRate(), DEFAULT_TT_COM);
    trajGenFoot     = new minJerkTrajGen(DEFAULT_KP_FOOT.rows(),    getRate(), DEFAULT_TT_FOOT);
    trajGenPosture  = new minJerkTrajGen(DEFAULT_KP_POSTURE.rows(), getRate(), DEFAULT_TT_POSTURE);
    
    // resize vectors that are not fixed-size
    kp_posture.resize(ICUB_DOFS);
    activeJoints.resize(ICUB_DOFS);

    // link module rpc parameters to member variables
    assert(paramHelper->linkParam(PARAM_ID_KP_COM,              kp_com.data()));
    assert(paramHelper->linkParam(PARAM_ID_KP_FOOT,             kp_foot.data()));
    assert(paramHelper->linkParam(PARAM_ID_KP_POSTURE,          kp_posture.data()));
    assert(paramHelper->linkParam(PARAM_ID_TRAJ_TIME_COM,       &tt_com));
    assert(paramHelper->linkParam(PARAM_ID_TRAJ_TIME_FOOT,      &tt_foot));
    assert(paramHelper->linkParam(PARAM_ID_TRAJ_TIME_POSTURE,   &tt_posture));
    assert(paramHelper->linkParam(PARAM_ID_ACTIVE_JOINTS,       activeJoints.data()));
    assert(paramHelper->linkParam(PARAM_ID_SUPPORT_PHASE,       &supportPhase));
    assert(paramHelper->linkParam(PARAM_ID_PINV_DAMP,           &pinvDamp));
    // link module input streaming parameters to member variables
    assert(paramHelper->linkParam(PARAM_ID_XDES_COM,            xd_com.data()));
    assert(paramHelper->linkParam(PARAM_ID_XDES_FOOT,           xd_foot.data()));
    assert(paramHelper->linkParam(PARAM_ID_QDES,                qd.data()));
    // link module output streaming parameters to member variables
    assert(paramHelper->linkParam(PARAM_ID_XREF_COM,            xr_com.data()));
    assert(paramHelper->linkParam(PARAM_ID_XREF_FOOT,           xr_foot.data()));
    assert(paramHelper->linkParam(PARAM_ID_QREF,                qr.data()));
    assert(paramHelper->linkParam(PARAM_ID_X_COM,               x_com.data()));
    assert(paramHelper->linkParam(PARAM_ID_X_FOOT,              x_foot.data()));
    assert(paramHelper->linkParam(PARAM_ID_Q,                   q.data()));
    
    // Register callbacks for some module parameters
    assert(paramHelper->registerParamCallback(PARAM_ID_TRAJ_TIME_COM,       this));
    assert(paramHelper->registerParamCallback(PARAM_ID_TRAJ_TIME_FOOT,      this));
    assert(paramHelper->registerParamCallback(PARAM_ID_TRAJ_TIME_POSTURE,   this));
    assert(paramHelper->registerParamCallback(PARAM_ID_ACTIVE_JOINTS,       this));
    assert(paramHelper->registerParamCallback(PARAM_ID_SUPPORT_PHASE,       this));

    // Register callbacks for some module commands
    assert(paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    assert(paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));
    
    return true;
}

//*************************************************************************************************************************
void LocomotionThread::run()
{
    paramHelper->lock();
    paramHelper->readStreamParams();

    Time::delay(0.005);
    
    paramHelper->sendStreamParams();
    paramHelper->unlock();
}

//*************************************************************************************************************************
void LocomotionThread::threadRelease()
{
    if(trajGenCom)      delete trajGenCom;
    if(trajGenFoot)     delete trajGenFoot;
    if(trajGenPosture)  delete trajGenPosture;
}

//*************************************************************************************************************************
void LocomotionThread::parameterUpdated(const ParamDescription &pd)
{
    switch(pd.id)
    {
    case PARAM_ID_TRAJ_TIME_COM: 
        trajGenCom->setT(tt_com); sendMsg("Traj time com changed to "+toString(tt_com), MSG_DEBUG); break;
    case PARAM_ID_TRAJ_TIME_FOOT: 
        trajGenFoot->setT(tt_foot); sendMsg("Traj time foot changed to "+toString(tt_foot), MSG_DEBUG); break;
    case PARAM_ID_TRAJ_TIME_POSTURE: 
        trajGenPosture->setT(tt_posture); sendMsg("Traj time posture changed to "+toString(tt_posture), MSG_DEBUG); break;
    case PARAM_ID_ACTIVE_JOINTS: 
        sendMsg("Active joints changed to "+toString(activeJoints), MSG_DEBUG); break;
    case PARAM_ID_SUPPORT_PHASE: 
        sendMsg("Support phase changed to "+toString(supportPhase), MSG_DEBUG); break;
    default:
        sendMsg("A callback is registered but not managed for the parameter "+pd.name, MSG_WARNING);
    }
}

//*************************************************************************************************************************
void LocomotionThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case COMMAND_ID_START:
        sendMsg("Starting the controller.", MSG_INFO); break;
    case COMMAND_ID_STOP:
        sendMsg("Stopping the controller.", MSG_INFO); break;
    default:
        sendMsg("A callback is registered but not managed for the command "+cd.name, MSG_WARNING);
    }
}

//*************************************************************************************************************************
void LocomotionThread::sendMsg(const string &s, MsgType type)
{
    if(type>=MSG_DEBUG)
        printf("[LocomotionThread] %s\n", s.c_str());
}