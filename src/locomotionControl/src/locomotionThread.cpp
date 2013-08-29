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

#include <locomotion/locomotionThread.h>
#include <yarp/os/Time.h>
#include <yarp/math/SVD.h>

using namespace locomotion;
using namespace yarp::math;

//*************************************************************************************************************************
LocomotionThread::LocomotionThread(string _name, string _robotName, int _period, ParamHelperServer *_ph, wholeBodyInterface *_wbi)
    :  RateThread(_period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi)
{

}

//*************************************************************************************************************************
bool LocomotionThread::threadInit()
{
    Vector zxx(2);

    assert(robot->getLinkId("r_foot", LINK_ID_RIGHT_FOOT));
    assert(robot->getLinkId("l_foot", LINK_ID_LEFT_FOOT));
    comLinkId           = 0; // iWholeBodyModel::COM_LINK_ID; // use 0 until Silvio implements COM kinematics
    
    // resize Eigen vectors that are not fixed-size
    
    // resize all Yarp vectors
    x_com.resize(DEFAULT_XDES_COM.size(), 0.0);          // measured pos
    x_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);        // measured pos
    q.resize(DEFAULT_QDES.size(), 0.0);                  // measured pos
    xd_com.resize(DEFAULT_XDES_COM.size(), 0.0);         // desired pos
    xd_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);       // desired pos
    qd.resize(DEFAULT_QDES.size(), 0.0);                 // desired pos
    xr_com.resize(DEFAULT_XDES_COM.size(), 0.0);         // reference pos
    xr_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);       // reference pos
    qr.resize(DEFAULT_QDES.size(), 0.0);                 // reference pos
    dxr_com.resize(DEFAULT_XDES_COM.size(), 0.0);        // reference vel
    dxr_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);      // reference vel
    dqr.resize(DEFAULT_QDES.size(), 0.0);                // reference vel
    dxc_com.resize(DEFAULT_XDES_COM.size(), 0.0);        // commanded vel
    dxc_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);      // commanded vel
    dqc.resize(DEFAULT_QDES.size(), 0.0);                // commanded vel
    kp_com.resize(DEFAULT_XDES_COM.size(), 0.0);         // proportional gain
    kp_foot.resize(DEFAULT_XDES_FOOT.size(), 0.0);       // proportional gain
    kp_posture.resize(DEFAULT_QDES.size(), 0.0);         // proportional gain
    H_w2b = eye(4,4);

    // map Yarp vectors to Eigen vectors
    dxc_comE.Map(dxc_com.data());
    dxc_footE.Map(dxc_foot.data());
    dqcE.Map(dqc.data());

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
    assert(paramHelper->linkParam(PARAM_ID_H_W2B,               H_w2b.data()));
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

    // read robot status
    if(!readRobotStatus(true))
        return false;

    // create and initialize trajectory generators
    trajGenCom      = new minJerkTrajGen(x_com,     getRate()*1e-3, DEFAULT_TT_COM);
    trajGenFoot     = new minJerkTrajGen(x_foot,    getRate()*1e-3, DEFAULT_TT_FOOT);
    trajGenPosture  = new minJerkTrajGen(q,         getRate()*1e-3, DEFAULT_TT_POSTURE);

    printf("\n\n");
    return true;
}

//*************************************************************************************************************************
void LocomotionThread::run()
{
    paramHelper->lock();
    paramHelper->readStreamParams();

    readRobotStatus();              // read encoders, compute positions and Jacobians
    updateReferenceTrajectories();  // compute reference trajectories
    
    dxc_com     = dxr_com   + kp_com    *(xr_com  - x_com);
    dxc_foot    = dxr_foot  + kp_foot   *(xr_foot - x_foot);
    dqc         = dqr       + kp_posture*(qr      - q);

    printf("\n*********************************************************************\n");
    printf("x foot:            %s\n", x_foot.toString(2).c_str());
    printf("x ref foot:        %s\n", xr_foot.toString(2).c_str());
    printf("x des foot:        %s\n", xd_foot.toString(2).c_str());
    printf("dx ref foot:       %s\n", dxr_foot.toString(2).c_str());
    printf("dx foot commanded: %s\n", dxc_foot.toString(2).c_str());
    
    VectorNd dqMotors = solveTaskHierarchy();   // prioritized velocity control

    paramHelper->sendStreamParams();
    paramHelper->unlock();
}

//*************************************************************************************************************************
bool LocomotionThread::readRobotStatus(bool blockingRead)
{
    // read joint angles
    bool res = robot->getQ(q.data(), blockingRead);

    // select which foot to control
    footLinkId = supportPhase==SUPPORT_LEFT ? LINK_ID_RIGHT_FOOT : LINK_ID_LEFT_FOOT;
    // base orientation conversion
    //Vector qu = dcm2quaternion(H_w2b.submatrix(0,2,0,2));
    Vector qu = dcm2axis(H_w2b.submatrix(0,2,0,2));     // temporarely use angle/axis notation
    xBase[0]=H_w2b(0,3);    xBase[1]=H_w2b(1,3);    xBase[2]=H_w2b(2,3);
    xBase[3]=qu[0];         xBase[4]=qu[1];         xBase[5]=qu[2];         xBase[6]=qu[3];

    // forward kinematics
    robot->forwardKinematics(q.data(), xBase.data(), footLinkId,    x_foot.data());
    robot->forwardKinematics(q.data(), xBase.data(), comLinkId,     x_com.data());
    // compute Jacobians of both feet and CoM
    robot->computeJacobian(q.data(), xBase.data(), LINK_ID_RIGHT_FOOT,  JfootR.data());
    robot->computeJacobian(q.data(), xBase.data(), LINK_ID_LEFT_FOOT,   JfootL.data());
    robot->computeJacobian(q.data(), xBase.data(), comLinkId,  Jcom.data());
    
    Jfoot = supportPhase==SUPPORT_LEFT ? JfootL : JfootR;
    int k = supportPhase==SUPPORT_DOUBLE ? 12 : 6;
    int n = q.size();
    Jc.resize(k, n);
    if(supportPhase==SUPPORT_DOUBLE)
    {
        Jc.topLeftCorner(6,n)       = JfootR;
        Jc.bottomLeftCorner(6,n)    = JfootL;
    }
    else if(supportPhase==SUPPORT_LEFT)
        Jc = JfootL;
    else
        Jc = JfootR;

    //cout<< "J foot 1:\n" << fixed<< setw(4)<< setprecision(1)<< Jfoot.block(0,0,6,5) <<endl;
    //cout<< "J foot 2:\n" << fixed<< setw(4)<< setprecision(1)<< Jfoot.block(0,5,6,15) <<endl;
    
    return res;
}

//*************************************************************************************************************************
bool LocomotionThread::updateReferenceTrajectories()
{
    trajGenCom->computeNextValues(xd_com);
    trajGenFoot->computeNextValues(xd_foot);
    trajGenPosture->computeNextValues(qd);
    xr_com      = trajGenCom->getPos();
    xr_foot     = trajGenFoot->getPos();
    qr          = trajGenPosture->getPos();
    dxr_com     = trajGenCom->getVel();
    dxr_foot    = trajGenFoot->getVel();
    dqr         = trajGenPosture->getVel();
    return true;
}

//*************************************************************************************************************************
VectorNd LocomotionThread::solveTaskHierarchy()
{
    int k = Jc.rows();
    int n = Jc.cols();
    VectorNd dqDes;
    dqDes.setZero();
    // CONTACT CONSTRAINTS
    MatrixXd Jc_pinv(n, k), Nc(n,n);
    VectorXd svJc(k);
    
    pinvTrunc(Jc, PINV_TOL, Jc_pinv);
    Nc.setIdentity();
    Nc -= Jc_pinv*Jc;
    //N = Nc;

    //// FORCE TASK 1
    //J = hier.f1.J*N;
    //pinvDampTrunc( J, J_pinv, J_pinvD, svJf1, PINV_TOL, hier.f1.pinvDamp, U, V, Spinv, SpinvD);
    //dqDesF1 = J_pinvD*(dxF1Des - hier.f1.J*dqDes);
    //dqDes += ddqDesF1;
    //N -= J_pinv*J;
    //// POSITION TASK 2
    //J = hier.p2.J * N;
    //pinvDampTrunc(J, J_pinv, J_pinvD, svJ2, PINV_TOL, hier.p2.pinvDamp, U, V, Spinv, SpinvD);
    //dqDes2 = J_pinvD*(dx2Des - hier.p2.J*dqDes);
    //dqDes += dqDes2;
    //N -= J_pinv*J;
    //// POSITION TASK 1
    //J = hier.p1.J * N;
    //pinvDampTrunc(J, J_pinv, J_pinvD, svJ1, PINV_TOL, hier.p1.pinvDamp, U, V, Spinv, SpinvD);
    //dqDes1 = J_pinvD*(dx1Des - hier.p1.J*dqDes);
    //dqDes += dqDes1;
    //N -= J_pinv*J;
    //// POSTURE TASK
    //dqDes += N*(dqDesPost - dqDes);

    return dqDes;
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
