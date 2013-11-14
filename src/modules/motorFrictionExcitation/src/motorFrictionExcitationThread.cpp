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
    :  RateThread(_period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi), 
    dxc_comE(0), dxc_footE(0), dqcE(0,0), qDegE(0,0)
{
    status = EXCITATION_OFF;
    printCountdown = 0;
}

//*************************************************************************************************************************
bool MotorFrictionExcitationThread::threadInit()
{
    YARP_ASSERT(robot->getLinkId("r_sole", LINK_ID_RIGHT_FOOT)); // 41
    YARP_ASSERT(robot->getLinkId("l_sole", LINK_ID_LEFT_FOOT));  // 33
    comLinkId           = iWholeBodyModel::COM_LINK_ID;

    // I must count the nonzero entries of activeJoints before calling numberOfJointsChanged (to know _n)
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_ACTIVE_JOINTS,       activeJoints.data()));
    // I must know the support phase before calling numberOfConstraintsChanged (to know the number of constraints)
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SUPPORT_PHASE,       &supportPhase));
    numberOfJointsChanged();
    numberOfConstraintsChanged();
    
    // resize all Yarp vectors
    x_com.resize(DEFAULT_XDES_COM.size(), 0.0);         // measured pos
    x_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);       // measured pos

    xd_com.resize(DEFAULT_XDES_COM.size(), 0.0);        // desired pos
    xd_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);      // desired pos
    qd.resize(ICUB_DOFS, 0.0);                          // desired pos (all joints)

    xr_com.resize(DEFAULT_XDES_COM.size(), 0.0);        // reference pos
    xr_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);      // reference pos
    qr.resize(ICUB_DOFS, 0.0);                          // reference pos

    dxr_com.resize(DEFAULT_XDES_COM.size(), 0.0);       // reference vel
    dxr_foot.resize(6, 0.0);                            // reference vel
    dqr.resize(ICUB_DOFS, 0.0);                         // reference vel

    dxc_com.resize(DEFAULT_XDES_COM.size(), 0.0);       // commanded vel
    dxc_foot.resize(6, 0.0);                            // commanded vel

    kp_com.resize(DEFAULT_XDES_COM.size(), 0.0);        // proportional gain
    kp_foot.resize(6, 0.0);                             // proportional gain
    kp_posture.resize(ICUB_DOFS, 0.0);                  // proportional gain
    //H_w2b = eye(4,4);

    // resize all Eigen vectors
    ftSens.resize(12); ftSens.setZero();

    // map Yarp vectors to Eigen vectors
    new (&dxc_comE)     Map<Vector2d>(dxc_com.data());
    new (&dxc_footE)    Map<Vector6d>(dxc_foot.data());

    // link module rpc parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KP_COM,              kp_com.data()));    // constant size
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KP_FOOT,             kp_foot.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_KP_POSTURE,          kp_posture.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TRAJ_TIME_COM,       &tt_com));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TRAJ_TIME_FOOT,      &tt_foot));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_TRAJ_TIME_POSTURE,   &tt_posture));
    // link module input streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_XDES_COM,            xd_com.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_XDES_FOOT,           xd_foot.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QDES,                qd.data()));        // constant size
#ifndef COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_H_W2B,               H_w2b.data()));
#endif
    // link module output streaming parameters to member variables
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_XREF_COM,            xr_com.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_XREF_FOOT,           xr_foot.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QREF,                qr.data()));        // constant size
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_X_COM,               x_com.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_X_FOOT,              x_foot.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q,                   qDeg.data()));      // variable size
    
    // Register callbacks for some module parameters
    YARP_ASSERT(paramHelper->registerParamCallback(PARAM_ID_XDES_FOOT,           this));
    YARP_ASSERT(paramHelper->registerParamCallback(PARAM_ID_TRAJ_TIME_COM,       this));
    YARP_ASSERT(paramHelper->registerParamCallback(PARAM_ID_TRAJ_TIME_FOOT,      this));
    YARP_ASSERT(paramHelper->registerParamCallback(PARAM_ID_TRAJ_TIME_POSTURE,   this));
    YARP_ASSERT(paramHelper->registerParamCallback(PARAM_ID_ACTIVE_JOINTS,       this));
    YARP_ASSERT(paramHelper->registerParamCallback(PARAM_ID_SUPPORT_PHASE,       this));

    // Register callbacks for some module commands
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));

#ifdef COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    Ha.R = Rotation(0,0,1, 0,-1,0, 1,0,0);   // rotation to align foot Z axis with gravity, Ha=[0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1]
#endif
    normalizeFootOrientation();

    // read robot status (to be done before initializing trajectory generators)
    if(!readRobotStatus(true))
        return false;

    // create and initialize trajectory generators
    trajGenCom      = new minJerkTrajGen(2,         getRate()*1e-3, DEFAULT_TT_COM);
    trajGenFoot     = new minJerkTrajGen(7,         getRate()*1e-3, DEFAULT_TT_FOOT);
    trajGenPosture  = new minJerkTrajGen(ICUB_DOFS, getRate()*1e-3, DEFAULT_TT_POSTURE);

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
        
    //sendMsg("Time to compute Jacobians and FK: "+toString(Time::now()-t0), MSG_INFO);
    //sendMsg("ft sens: "+toString(ftSens.transpose(),1), MSG_DEBUG);
    //sendMsg("q rad: "+string(qRad.toString(2)), MSG_INFO);
    //sendMsg("q deg: "+string(qDeg.toString(2)), MSG_INFO);
    //sendMsg("H_w2b:\n"+string(H_w2b.toString(2)), MSG_INFO);
    //sendMsg("xBase:\n"+toString(xBase.transpose(),2), MSG_INFO);
    //sendMsg("footLinkId: "+toString(footLinkId), MSG_INFO);
    //sendMsg("x com: "+string(x_com.toString(2)), MSG_INFO);
    //cout<< "R foot vel: "<< setprecision(2)<< (JfootR*dq).norm()<< endl; //.transpose().format(matrixPrintFormat)<< endl;
    //cout<< "L foot vel: "<< setprecision(2)<< (JfootL*dq).norm()<< endl; //transpose().format(matrixPrintFormat)<< endl;
    //sendMsg("Jc (Rfoot up, Lfoot down):\n"+toString(Jc,2), MSG_DEBUG);
    //sendMsg("Jcom:\n"+toString(solver->com.A,2), MSG_DEBUG);
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
    trajGenCom->init(x_com);                // initialize trajectory generators
    trajGenFoot->init(x_foot);
    trajGenPosture->init(qRad);
    status = EXCITATION_ON;                 // set thread status to "on"
    robot->setControlMode(CTRL_MODE_VEL);
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
void MotorFrictionExcitationThread::numberOfConstraintsChanged()
{

}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::numberOfJointsChanged()
{
    
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::updateSelectionMatrix()
{
    
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::normalizeFootOrientation()
{
    double axisNorm = norm3d(&(xd_foot[3]));
    xd_foot[3] /= axisNorm;
    xd_foot[4] /= axisNorm;
    xd_foot[5] /= axisNorm;
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::threadRelease()
{
    if(trajGenCom)      delete trajGenCom;
    if(trajGenFoot)     delete trajGenFoot;
    if(trajGenPosture)  delete trajGenPosture;
}

//*************************************************************************************************************************
void MotorFrictionExcitationThread::parameterUpdated(const ParamDescription &pd)
{
    switch(pd.id)
    {
    case PARAM_ID_XDES_FOOT:
        normalizeFootOrientation(); break;
    case PARAM_ID_TRAJ_TIME_COM: 
        trajGenCom->setT(tt_com); break;
    case PARAM_ID_TRAJ_TIME_FOOT: 
        trajGenFoot->setT(tt_foot); break;
    case PARAM_ID_TRAJ_TIME_POSTURE: 
        trajGenPosture->setT(tt_posture); break;
    case PARAM_ID_ACTIVE_JOINTS: 
        numberOfJointsChanged(); break;
    case PARAM_ID_SUPPORT_PHASE: 
        numberOfConstraintsChanged(); break;
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
