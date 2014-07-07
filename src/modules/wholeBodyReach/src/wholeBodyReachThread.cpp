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

#include <wholeBodyReach/wholeBodyReachThread.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/math/SVD.h>
#include <Eigen/Cholesky>


using namespace wholeBodyReach;
using namespace yarp::math;
using namespace wbiIcub;


//*************************************************************************************************************************
WholeBodyReachThread::WholeBodyReachThread(string name, string robotName, int period,
                                           ParamHelperServer *ph, wholeBodyInterface *wbi)
    : RateThread(period),
    _tasks(GRASP_HAND_LINK_NAME, SUPPORT_FOREARM_LINK_NAME, LEFT_FOOT_LINK_NAME,
           RIGHT_FOOT_LINK_NAME, period*1e-3, wbi),
    _solver(wbi, DEFAULT_USE_NULLSPACE_BASE),
    _name(name), _robotName(robotName), _paramHelper(ph), _robot(wbi)
{
    _status = WHOLE_BODY_REACH_OFF;
}

//*************************************************************************************************************************
bool WholeBodyReachThread::threadInit()
{
    YARP_ASSERT(_robot->getLinkId(RIGHT_FOOT_LINK_NAME.c_str(), LINK_ID_RIGHT_FOOT)); // 41
    YARP_ASSERT(_robot->getLinkId(LEFT_FOOT_LINK_NAME.c_str(),  LINK_ID_LEFT_FOOT));  // 33

    // I must know the support phase before calling numberOfConstraintsChanged (to know the number of constraints)
    YARP_ASSERT(_paramHelper->linkParam(PARAM_ID_SUPPORT_PHASE,       &_supportPhase));
    _n = _robot->getJointList().size();
    cout<< "The robot has "<< _n<< " degrees of freedom\n";
    _k = 12;
    
    // resize all vectors
    _qjDeg.setZero(_n);
    _tauDes.setZero(_n);
    _JfootR.resize(NoChange, _n+6);
    _JfootL.resize(NoChange, _n+6);
    _Jc.resize(_k, _n+6);
    _svdJcb = JacobiSVD<MatrixRXd>(_k, 6, ComputeThinU | ComputeThinV);
    _robotState.qJ.setZero(_n);     // joint positions (rad)
    _robotState.dqJ.setZero(_n);    // joint velocities (rad/s)
    _robotState.dq.setZero(_n+6);   // base+joint velocities
    
    _robotState.g(0) = 0.0;
    _robotState.g(1) = 0.0;
    _robotState.g(2) = -9.81;
    
    // setup the stack of tasks
    _solver.setMomentumTask(_tasks.momentum);
    _solver.setPostureTask(_tasks.posture);
    _solver.setJointLimitTask(_tasks.jointLimits);
    _solver.addConstraint(_tasks.leftFoot);
    _solver.addConstraint(_tasks.rightFoot);
    _solver.pushEqualityTask(_tasks.supportForearm);
    
    _solver.linkParameterToVariable(wbiStackOfTasks::NUMERICAL_DAMPING,  _paramHelper, PARAM_ID_NUM_DAMP);
    _solver.linkParameterToVariable(wbiStackOfTasks::USE_NULLSPACE_BASE, _paramHelper, PARAM_ID_USE_NULLSPACE_BASE);

    // link module rpc parameters to member variables
    _tasks.momentum.linkParameterKp(         _paramHelper, PARAM_ID_KP_MOMENTUM);
    _tasks.supportForearm.linkParameterKp(   _paramHelper, PARAM_ID_KP_FOREARM);
    _tasks.graspHand.linkParameterKp(        _paramHelper, PARAM_ID_KP_HAND);
    _tasks.posture.linkParameterKp(          _paramHelper, PARAM_ID_KP_POSTURE);
    
    _tasks.momentum.linkParameterTrajectoryDuration(       _paramHelper, PARAM_ID_TRAJ_TIME_MOMENTUM);
    _tasks.supportForearm.linkParameterTrajectoryDuration( _paramHelper, PARAM_ID_TRAJ_TIME_FOREARM);
    _tasks.graspHand.linkParameterTrajectoryDuration(      _paramHelper, PARAM_ID_TRAJ_TIME_HAND);
    _tasks.posture.linkParameterTrajectoryDuration(        _paramHelper, PARAM_ID_TRAJ_TIME_POSTURE);
    
//    YARP_ASSERT(_paramHelper->linkParam(PARAM_ID_Q_MAX,               solver->qMax.data()));
//    YARP_ASSERT(_paramHelper->linkParam(PARAM_ID_Q_MIN,               solver->qMin.data()));
//    YARP_ASSERT(_paramHelper->linkParam(PARAM_ID_JNT_LIM_MIN_DIST,    &(solver->safetyThreshold)));
    
    // link module input streaming parameters to member variables
    _tasks.momentum.linkParameterComDes(         _paramHelper, PARAM_ID_XDES_COM);
    _tasks.supportForearm.linkParameterPoseDes(  _paramHelper, PARAM_ID_XDES_FOREARM);
    _tasks.graspHand.linkParameterPoseDes(       _paramHelper, PARAM_ID_XDES_HAND);
    _tasks.posture.linkParameterPostureDes(      _paramHelper, PARAM_ID_QDES);
    
#ifndef COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    YARP_ASSERT(_paramHelper->linkParam(PARAM_ID_H_W2B,   _H_w2b.data()));
#endif
//    // link module output streaming parameters to member variables
//    YARP_ASSERT(_paramHelper->linkParam(PARAM_ID_XREF_COM,            xr_com.data()));
//    YARP_ASSERT(_paramHelper->linkParam(PARAM_ID_XREF_FOOT,           xr_foot.data()));
//    YARP_ASSERT(_paramHelper->linkParam(PARAM_ID_QREF,                qr.data()));        // constant size
    
    _tasks.momentum.linkParameterCom(       _paramHelper, PARAM_ID_X_COM);
    _tasks.supportForearm.linkParameterPose(_paramHelper, PARAM_ID_X_FOREARM);
    _tasks.graspHand.linkParameterPose(     _paramHelper, PARAM_ID_X_HAND);
    _paramHelper->linkParam(                PARAM_ID_Q,   _qjDeg.data());
    
    // Register callbacks for some module parameters
    YARP_ASSERT(_paramHelper->registerParamValueChangedCallback(PARAM_ID_SUPPORT_PHASE,       this));

    // Register callbacks for some module commands
    YARP_ASSERT(_paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    YARP_ASSERT(_paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));

#ifdef COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    _Ha.R = Rotation(0,0,1, 0,-1,0, 1,0,0);   // rotation to align foot Z axis with gravity, Ha=[0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1]
#endif

    // read _robot status (to be done before initializing trajectory generators)
#ifdef DO_NOT_USE_WHOLE_BODY_STATE_INTERFACE
    _sensors = new icubWholeBodySensors((_name+"_sensor").c_str(), _robotName.c_str());
    _sensors->addSensors(SENSOR_ENCODER, ICUB_MAIN_JOINTS);
    if(!_sensors->init())
    {
        printf("Initialization of sensor interface failed");
        return false;
    }
    _dqFilt = new AWLinEstimator(16, 1);
    _dqJ_yarp.resize(_n);
    _qJ_yarp.resize(_n);
    _qJStamps.resize(_n);
#endif
    
    if(!readRobotStatus(true))
        return false;

    printf("\n\n");
    return true;
}

//*************************************************************************************************************************
void WholeBodyReachThread::run()
{
    _paramHelper->lock();
    _paramHelper->readStreamParams();

    readRobotStatus();                      // read encoders, compute positions and Jacobians
    
    _solver.computeSolution(_robotState, _tauDes);   // compute desired joint torques

    if(_status==WHOLE_BODY_REACH_ON)
    {
        if(areDesiredJointTorquesTooLarge())    // check desired joint torques are not too large
        {
            preStopOperations();            // stop the controller
            cout<<"\n***** ERROR: CONTROLLER STOPPED BECAUSE DESIRED JOINT TORQUES ARE TOO LARGE: "
                <<toString(_tauDes.transpose(),1)<<endl;
        }
        else
            _robot->setControlReference(_tauDes.data());
    }

    _paramHelper->sendStreamParams();
    _paramHelper->unlock();

    sendMsg("tauDes \t"+toString(_tauDes.transpose(),1));
    getLogger().countdown();
}

//*************************************************************************************************************************
bool WholeBodyReachThread::readRobotStatus(bool blockingRead)
{
    // read joint angles
    bool res = true;

    // temporary replacement of _robot->getEstimate because it's too slow
#ifdef DO_NOT_USE_WHOLE_BODY_STATE_INTERFACE
    res = res && _sensors->readSensors(SENSOR_ENCODER, _robotState.qJ.data(), _qJStamps.data(), true);
    for(int i=0; i<_n; i++)
        _qJ_yarp(i) = _robotState.qJ(i);
    AWPolyElement el;
    el.data = _qJ_yarp;
    el.time = _qJStamps[0];
    _dqJ_yarp = 2.0*_dqFilt->estimate(el);
    for(int i=0; i<_n; i++)
        _robotState.dqJ(i) = _dqJ_yarp(i);
#else
    res = res && _robot->getEstimates(ESTIMATE_JOINT_POS,    _robotState.qJ.data(),     -1.0, blockingRead);
    res = res && _robot->getEstimates(ESTIMATE_JOINT_VEL,    _robotState.dqJ.data(),    -1.0, blockingRead);
#endif
    _qjDeg = CTRL_RAD2DEG*_robotState.qJ;

    // base orientation conversion
#ifdef COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    _robot->computeH(_robotState.qJ.data(), Frame(), LINK_ID_LEFT_FOOT, _H_base_leftFoot);
    _H_base_leftFoot = _H_base_leftFoot*_Ha;
    _H_base_leftFoot.setToInverse().get4x4Matrix(_H_w2b.data());    // homogeneous transformation from world (i.e. left foot) to base
#endif
    _robotState.xBase.set4x4Matrix(_H_w2b.data());

    // compute Jacobians of both feet
    if(_supportPhase==WBR_SUPPORT_DOUBLE)
    {
        res = res && _robot->computeJacobian(_robotState.qJ.data(), _robotState.xBase, LINK_ID_RIGHT_FOOT,  _JfootR.data());
        res = res && _robot->computeJacobian(_robotState.qJ.data(), _robotState.xBase, LINK_ID_LEFT_FOOT,   _JfootL.data());
        _Jc.topRows<6>()     = _JfootR;
        _Jc.bottomRows<6>()  = _JfootL;
    }
    
    // estimate base velocity from joint velocities and constraint Jacobian Jc
    _svdJcb.compute(_Jc.leftCols<6>(), ComputeThinU | ComputeThinV);
    _robotState.vBase = _svdJcb.solve(_Jc.rightCols(_n)*_robotState.dqJ);

    // copy base and joint velocities into _robotState.dq
    _robotState.dq.head<6>() = _robotState.vBase;
    _robotState.dq.tail(_n) = _robotState.dqJ;
    
    return res;
}


//*************************************************************************************************************************
bool WholeBodyReachThread::areDesiredJointTorquesTooLarge()
{
    for(int i=0; i<_n; i++)
        if(_tauDes(i)> TAU_MAX || _tauDes(i)<-TAU_MAX)
            return true;
    return false;
}

//*************************************************************************************************************************
void WholeBodyReachThread::preStartOperations()
{
    // no need to lock because the mutex is already locked
    readRobotStatus(true);
    // initialize trajectory generators
    _solver.init(_robotState);
    _status = WHOLE_BODY_REACH_ON;                 // set thread status to "on"
    _robot->setControlMode(CTRL_MODE_TORQUE);
}

//*************************************************************************************************************************
void WholeBodyReachThread::preStopOperations()
{
    // no need to lock because the mutex is already locked
//    VectorXd dqMotors = VectorXd::Zero(_n);
//    _robot->setControlReference(dqMotors.data());       // stop joint motors
    _robot->setControlMode(CTRL_MODE_POS);              // set position control mode
    _status = WHOLE_BODY_REACH_OFF;                            // set thread status to "off"
}

//*************************************************************************************************************************
void WholeBodyReachThread::numberOfConstraintsChanged()
{
//    _k = supportPhase==SUPPORT_DOUBLE ? 12 : 6;     // current number of constraints
//    _solver->resize(_k, _n+6);
}


//*************************************************************************************************************************
//void WholeBodyReachThread::normalizeFootOrientation()
//{
//    double axisNorm = norm3d(&(xd_foot[3]));
//    xd_foot[3] /= axisNorm;
//    xd_foot[4] /= axisNorm;
//    xd_foot[5] /= axisNorm;
//}

//*************************************************************************************************************************
void WholeBodyReachThread::threadRelease()
{
}

//*************************************************************************************************************************
void WholeBodyReachThread::parameterUpdated(const ParamProxyInterface *pd)
{
    switch(pd->id)
    {
    case PARAM_ID_XDES_FOREARM:
//        normalizeFootOrientation(); break;
    case PARAM_ID_TRAJ_TIME_MOMENTUM:
    case PARAM_ID_TRAJ_TIME_FOREARM:
    case PARAM_ID_TRAJ_TIME_HAND:
    case PARAM_ID_TRAJ_TIME_POSTURE:
//        trajGenPosture->setT(tt_posture);
            break;
    case PARAM_ID_SUPPORT_PHASE:
        numberOfConstraintsChanged(); break;
    default:
        sendMsg("A callback is registered but not managed for the parameter "+pd->name, MSG_ERROR);
    }
}

//*************************************************************************************************************************
void WholeBodyReachThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
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
        sendMsg("A callback is registered but not managed for the command "+cd.name, MSG_ERROR);
    }
}

//*************************************************************************************************************************
void WholeBodyReachThread::startController()
{
    _paramHelper->lock();
    preStartOperations();
    _paramHelper->unlock();
}

//*************************************************************************************************************************
void WholeBodyReachThread::sendMsg(const string &s, MsgType type)
{
    getLogger().sendMsg("[WBRThread] "+s, type);
}


// JOINT SPACE INVERSE DYNAMICS
//MatrixRXd Jc = solver->constraints.A;
//MatrixRXd Nc        = nullSpaceProjector(Jc, PINV_TOL);
//MatrixRXd NcSTpinvD = pinvDampedEigen(Nc.rightCols(_n), 1e-8);
//MatrixRXd Jcpinv    = pinvDampedEigen(Jc, PINV_TOL);
//VectorXd dJcdq(_k);
//_robot->computeDJdq(qRad.data(), xBase, dqJ.data(), dq.data(), LINK_ID_RIGHT_FOOT, dJcdq.data());
//_robot->computeDJdq(qRad.data(), xBase, dqJ.data(), dq.data(), LINK_ID_LEFT_FOOT,  dJcdq.data()+6);
//VectorXd ddqDes1   = -Jcpinv*dJcdq;
//VectorXd ddqDes = ddqDes1 + NcSTpinvD.transpose()*(dqcE - ddqDes1.tail(_n));
//VectorXd  tauDes    = NcSTpinvD * Nc * (M*ddqDes + h);

//// check that tauDes gives the desired accelerations
//MatrixRXd Minv = M.llt().solve(MatrixRXd::Identity(_n+6,_n+6));
//MatrixRXd Lambda_c_inv = (Jc*Minv*Jc.transpose());
//VectorXd tau_Np6 = VectorXd::Zero(_n+6);
//tau_Np6.tail(_n) = tauDes;
//VectorXd b = Jc*Minv*(h - tau_Np6) - dJcdq;
//VectorXd fc = Lambda_c_inv.llt().solve(b);
//VectorXd ddq = Minv*(tau_Np6 - h + Jc.transpose()*fc);
//sendMsg("ddq-ddqDes: "+toString((ddq-ddqDes).tail(_n).transpose(),1), MSG_DEBUG);
//sendMsg("M*ddq+h-J^T*f-tau: "+toString((M*ddq+h-Jc.transpose()*fc-tau_Np6).transpose(),1), MSG_DEBUG);
