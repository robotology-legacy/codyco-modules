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
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/math/SVD.h>


using namespace locomotion;
using namespace yarp::math;
using namespace yarpWbi;


//*************************************************************************************************************************
LocomotionThread::LocomotionThread(string _name, string _robotName, int _period, ParamHelperServer *_ph, wholeBodyInterface *_wbi)
    :  RateThread(_period), name(_name), robotName(_robotName), paramHelper(_ph), robot(_wbi),
    dxc_comE(0), dxc_footE(0), dqcE(0,0), qDegE(0,0)
{
    status = LOCOMOTION_OFF;
    printCountdown = 0;
    solver = NULL;
}

//*************************************************************************************************************************
bool LocomotionThread::threadInit()
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
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_PINV_DAMP,           &(solver->pinvDamp)));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_MAX,               solver->qMax.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_Q_MIN,               solver->qMin.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_JNT_LIM_MIN_DIST,    &(solver->safetyThreshold)));
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
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_XDES_FOOT,           this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_TRAJ_TIME_COM,       this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_TRAJ_TIME_FOOT,      this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_TRAJ_TIME_POSTURE,   this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_ACTIVE_JOINTS,       this));
    YARP_ASSERT(paramHelper->registerParamValueChangedCallback(PARAM_ID_SUPPORT_PHASE,       this));

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
    trajGenCom      = new minJerkTrajGen(2,         getRate()*1e-3, tt_com);
    trajGenFoot     = new minJerkTrajGen(7,         getRate()*1e-3, tt_foot);
    trajGenPosture  = new minJerkTrajGen(ICUB_DOFS, getRate()*1e-3, tt_posture);

    xd_com      = x_com;
    xd_foot     = x_foot;
    qd          = qDeg;

    printf("\n\n");
    return true;
}

//*************************************************************************************************************************
void LocomotionThread::run()
{
    paramHelper->lock();
    paramHelper->readStreamParams();

    readRobotStatus();                      // read encoders, compute positions and Jacobians
    if(status==LOCOMOTION_ON)
    {
        updateReferenceTrajectories();      // compute desired velocities for all tasks
        solver->solve(dqDes, qDegE);        // compute desired joint velocities

        if(areDesiredJointVelTooLarge())    // check desired joint velocities are not too large
        {
            preStopOperations();            // stop the controller
            cout<<"\n************ ERROR: CONTROLLER STOPPED BECAUSE DESIRED JOINT VELOCITIES ARE TOO LARGE: "<<toString(dqDes.transpose(),2)<<endl;
        }
        else
            robot->setControlReference(dqDes.data()); // send velocities to the joint motors

        sendMsg("Solver time: "+toString(solver->solverTime)+"; iterations: "+toString(solver->solverIterations)+"; blocked joints: "+toString(solver->getBlockedJointList()), MSG_INFO);
        sendMsg("dqDes: "+toString(1e3*dqDes.transpose(), 1), MSG_DEBUG);
    }

    paramHelper->sendStreamParams();
    paramHelper->unlock();

    printCountdown = (printCountdown>=PRINT_PERIOD) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)
}

//*************************************************************************************************************************
bool LocomotionThread::readRobotStatus(bool blockingRead)
{
    // read joint angles
    bool res =   robot->getEstimates(ESTIMATE_JOINT_POS,    qRad.data(),    -1.0, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_JOINT_VEL,    dqJ.data(),     -1.0, blockingRead);
    res = res && robot->getEstimates(ESTIMATE_FORCE_TORQUE_SENSOR, ftSens.data(),  -1.0, blockingRead);
    qDeg = CTRL_RAD2DEG*qRad;

    // base orientation conversion
#ifdef COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    robot->computeH(qRad.data(), Frame(), LINK_ID_LEFT_FOOT, H_base_leftFoot);
    H_base_leftFoot = H_base_leftFoot*Ha;
    H_base_leftFoot.setToInverse().get4x4Matrix(H_w2b.data());    // rototranslation from world (i.e. left foot) to robot base
#endif
    xBase.set4x4Matrix(H_w2b.data());

    // select which foot to control (when in double support, select the right foot)
    footLinkId = supportPhase==SUPPORT_RIGHT ? LINK_ID_LEFT_FOOT : LINK_ID_RIGHT_FOOT;

    // forward kinematics
    double x_com7d[7];
    res = res && robot->forwardKinematics(qRad.data(), xBase, footLinkId,    x_foot.data());
    res = res && robot->forwardKinematics(qRad.data(), xBase, comLinkId,     x_com7d);
    x_com(0) = x_com7d[0];
    x_com(1) = x_com7d[1];

    // compute Jacobians of both feet and CoM
    res = res && robot->computeJacobian(qRad.data(), xBase, LINK_ID_RIGHT_FOOT,  JfootR.data());
    res = res && robot->computeJacobian(qRad.data(), xBase, LINK_ID_LEFT_FOOT,   JfootL.data());
    res = res && robot->computeJacobian(qRad.data(), xBase, comLinkId,           Jcom_6xN.data());

    // convert Jacobians
    solver->com.A = Jcom_6xN.topRows<2>();  // we control just CoM projection on the ground
    if(supportPhase==SUPPORT_DOUBLE){       solver->foot.A.setZero();    solver->constraints.A.topRows<6>()=JfootR; solver->constraints.A.bottomRows<6>()=JfootL; }
    else if(supportPhase==SUPPORT_LEFT){    solver->foot.A=JfootR;       solver->constraints.A = JfootL; }
    else{                                   solver->foot.A=JfootL;       solver->constraints.A = JfootR; }

    // estimate base velocity from joint velocities
    Jcb = solver->constraints.A.leftCols<6>();
    svdJcb = Jcb.jacobiSvd(ComputeThinU | ComputeThinV);
    dq.head<6>() = svdJcb.solve(solver->constraints.A.rightCols(_n)*dqJ);
    dq.tail(_n) = dqJ;

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
bool LocomotionThread::updateReferenceTrajectories()
{
    trajGenCom->computeNextValues(xd_com);
    trajGenFoot->computeNextValues(xd_foot);
    trajGenPosture->computeNextValues(CTRL_DEG2RAD*qd);
    xr_com      = trajGenCom->getPos();
    xr_foot     = trajGenFoot->getPos();
    qr          = trajGenPosture->getPos(); // rad
    dxr_com     = trajGenCom->getVel();
    dxr_foot    = trajGenFoot->getVel();
    dqr         = trajGenPosture->getVel(); // rad/sec
    // for the orientation part of xr_foot, bypass the trajectory generator and set it to xd_foot
    xr_foot.setSubvector(3, xd_foot.subVector(3,6));

    dxc_com     = dxr_com   +  kp_com        * (xr_com  - x_com);
    dxc_foot    =/*dxr_foot+*/ kp_foot       * compute6DError(x_foot, xr_foot);  // temporarely remove feedforward velocity because it is 7d (whereas it should be 6d)
    dqc         = S*dqr     +  (S*kp_posture)* (S*qr    - qRad);

    solver->com.b = dxc_comE;
    solver->foot.b = dxc_footE;
    solver->posture.b = dqcE;
    return true;
}

//*************************************************************************************************************************
bool LocomotionThread::areDesiredJointVelTooLarge()
{
    for(int i=0; i<dqDes.size(); i++)
        if(dqDes(i)> DQ_MAX || dqDes(i)<-DQ_MAX)
            return true;
    return false;
}

//*************************************************************************************************************************
void LocomotionThread::preStartOperations()
{
    // no need to lock because the mutex is already locked
    readRobotStatus(true);                  // update com, foot and joint positions
    trajGenCom->init(x_com);                // initialize trajectory generators
    trajGenFoot->init(x_foot);
    trajGenPosture->init(qRad);
    status = LOCOMOTION_ON;                 // set thread status to "on"
    robot->setControlMode(CTRL_MODE_VEL);
    /*
    for(int i=0; i<13; i++)
        robot->setControlMode(CTRL_MODE_VEL, i);   // set position control mode
    for(int i=13; i<25; i++)
        robot->setControlMode(CTRL_MODE_VEL, i);   // set position control mode
    */
}

//*************************************************************************************************************************
void LocomotionThread::preStopOperations()
{
    // no need to lock because the mutex is already locked
    VectorXd dqMotors = VectorXd::Zero(_n);
    robot->setControlReference(dqMotors.data());      // stop joint motors
    robot->setControlMode(CTRL_MODE_POS);   // set position control mode
    status = LOCOMOTION_OFF;                // set thread status to "off"
}

//*************************************************************************************************************************
void LocomotionThread::numberOfConstraintsChanged()
{
    _k = supportPhase==SUPPORT_DOUBLE ? 12 : 6;     // current number of constraints
    solver->resize(_k, _n+6);
}

//*************************************************************************************************************************

void LocomotionThread::numberOfJointsChanged()
{
    LocalId lid;
    LocalIdList currentActiveJoints = robot->getJointList();
    for(int i=0; i<activeJoints.size(); i++)
    {
        lid = ICUB_MAIN_JOINTS.globalToLocalId(i);
        if(currentActiveJoints.containsId(lid))
        {
            if(activeJoints[i]==0)
                robot->removeJoint(lid);
        }
        else
        {
            if(activeJoints[i]==1)
                robot->addJoint(lid);
        }
    }

    _n = robot->getJointList().size();
    Jcom_6xN.resize(NoChange, _n+6);
    JfootR.resize(NoChange, _n+6);
    JfootL.resize(NoChange, _n+6);
    if(solver!=NULL)
        solver->resize(_k,_n+6);
    else
        solver = new LocomotionSolver(supportPhase==SUPPORT_DOUBLE ? 12 : 6,_n+6, PINV_TOL, 1e-4);
    solver->posture.A = MatrixXd::Zero(_n, _n+6);
    solver->posture.A.rightCols(_n) = MatrixXd::Identity(_n,_n);

    qRad.resize(_n, 0.0);                               // measured pos
    qDeg.resize(_n, 0.0);                               // measured pos
    new (&qDegE) Map<VectorXd>(qDeg.data(), _n);        // measured pos (Eigen vector)
    dq.resize(_n+6);                                    // measured vel (base + joints)
    dqJ.resize(_n);                                     // measured vel (joints only)
    dqc.resize(_n, 0.0);                                // commanded vel (Yarp vector)
    new (&dqcE) Map<VectorXd>(dqc.data(), _n);          // commanded vel (Eigen vector)
    dqDes.resize(_n);                                   // desired joint vel commanded to the motors
    kp_posture.resize(_n, 0.0);                         // proportional gain (rpc input parameter)
    // Note: qd, qr and dqr have constant size = ICUB_DOFS
    //if(!robot->getJointLimits(solver->qMin.data(), solver->qMax.data()))
    //    sendMsg("Error while reading joint limits.", MSG_ERROR);
    //else
    //{
    //    solver->qMin *= CTRL_RAD2DEG;   // convert from rad to deg
    //    solver->qMax *= CTRL_RAD2DEG;   // convert from rad to deg
    //    cout<< "qMin: "<<toString(solver->qMin.transpose(),0)<<"\nqMax: "<<toString(solver->qMax.transpose(),0)<<endl;
    //}
    updateSelectionMatrix();
}

//*************************************************************************************************************************
void LocomotionThread::updateSelectionMatrix()
{
    S.resize(_n, ICUB_DOFS);
    S.zero();
    int j=0;
    for(int i=0; i<ICUB_DOFS; i++)
    {
        if(activeJoints[i] != 0.0)
        {
            S(j,i) = 1.0;
            j++;
        }
    }
}

//*************************************************************************************************************************
void LocomotionThread::normalizeFootOrientation()
{
    double axisNorm = norm3d(&(xd_foot[3]));
    xd_foot[3] /= axisNorm;
    xd_foot[4] /= axisNorm;
    xd_foot[5] /= axisNorm;
}

//*************************************************************************************************************************
void LocomotionThread::threadRelease()
{
    if(trajGenCom)      delete trajGenCom;
    if(trajGenFoot)     delete trajGenFoot;
    if(trajGenPosture)  delete trajGenPosture;
}

//*************************************************************************************************************************
void LocomotionThread::parameterUpdated(const ParamProxyInterface *pd)
{
    switch(pd->id)
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
        sendMsg("A callback is registered but not managed for the parameter "+pd->name, MSG_WARNING);
    }
}

//*************************************************************************************************************************
void LocomotionThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
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
void LocomotionThread::startController()
{
    paramHelper->lock();
    preStartOperations();
    paramHelper->unlock();
}

//*************************************************************************************************************************
void LocomotionThread::sendMsg(const string &s, MsgType type)
{
    if(printCountdown==0 && type>=PRINT_MSG_LEVEL)
        printf("[LocomotionThread] %s\n", s.c_str());
}
