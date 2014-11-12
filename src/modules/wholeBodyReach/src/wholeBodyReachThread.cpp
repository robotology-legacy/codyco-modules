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
#include <wholeBodyReach/wholeBodyReachParams.h>
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
    _name(name), _robotName(robotName), _paramHelper(ph), _robot(wbi),
    _contactReader(name, robotName),
    _solver(wbi, DEFAULT_USE_NULLSPACE_BASE),
    _tasks(GRASP_HAND_LINK_NAME, SUPPORT_FOREARM_LINK_NAME, LEFT_FOOT_LINK_NAME,
           RIGHT_FOOT_LINK_NAME, period*1e-3, ICUB_FOOT_SIZE, wbi),
    _integrator(wbi)
{
    _time = 0.0;
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
    _dqjDeg.setZero(_n);
    _tauDes.setZero(_n);
    _JfootR.resize(NoChange, _n+6);
    _JfootL.resize(NoChange, _n+6);
    _Jc.resize(_k, _n+6);
    _svdJcb = JacobiSVD<MatrixRXd>(_k, 6, ComputeThinU | ComputeThinV);
    if(!_robotState.init(_n) || !_robotStateIntegrator.init(_n))
        return false;
    
    
    // setup the stack of tasks
    _solver.setMomentumTask(_tasks.momentum);
    _solver.setPostureTask(_tasks.posture);
    _solver.setJointLimitTask(_tasks.jointLimits);
    _solver.addConstraint(_tasks.leftFoot);
    _solver.addConstraint(_tasks.rightFoot);
    _solver.pushEqualityTask(_tasks.supportForearm);
    _solver.pushEqualityTask(_tasks.graspHand);
    
    _solver.linkParameterToVariable(wbiStackOfTasks::DYN_NUM_DAMP,       _paramHelper, PARAM_ID_DYN_DAMP);
    _solver.linkParameterToVariable(wbiStackOfTasks::CONSTR_NUM_DAMP,    _paramHelper, PARAM_ID_CONSTR_DAMP);
    _solver.linkParameterToVariable(wbiStackOfTasks::TASK_NUM_DAMP,      _paramHelper, PARAM_ID_TASK_DAMP);
    _solver.linkParameterToVariable(wbiStackOfTasks::USE_NULLSPACE_BASE, _paramHelper, PARAM_ID_USE_NULLSPACE_BASE);
    _solver.linkParameterToVariable(wbiStackOfTasks::CTRL_ALG,           _paramHelper, PARAM_ID_CTRL_ALGORITHM);

    // link module rpc parameters to member variables
    _tasks.momentum.linkParameterKp(         _paramHelper, PARAM_ID_KP_MOMENTUM);
    _tasks.supportForearm.linkParameterKp(   _paramHelper, PARAM_ID_KP_FOREARM);
    _tasks.graspHand.linkParameterKp(        _paramHelper, PARAM_ID_KP_HAND);
    _tasks.posture.linkParameterKp(          _paramHelper, PARAM_ID_KP_POSTURE);
    
    _tasks.momentum.linkParameterKd(         _paramHelper, PARAM_ID_KD_MOMENTUM);
    _tasks.supportForearm.linkParameterKd(   _paramHelper, PARAM_ID_KD_FOREARM);
    _tasks.graspHand.linkParameterKd(        _paramHelper, PARAM_ID_KD_HAND);
    _tasks.posture.linkParameterKd(          _paramHelper, PARAM_ID_KD_POSTURE);
    _tasks.posture.linkParameterKi(          _paramHelper, PARAM_ID_KI_POSTURE);
    
    _tasks.momentum.linkParameterTrajectoryDuration(       _paramHelper, PARAM_ID_TRAJ_TIME_MOMENTUM);
    _tasks.supportForearm.linkParameterTrajectoryDuration( _paramHelper, PARAM_ID_TRAJ_TIME_FOREARM);
    _tasks.graspHand.linkParameterTrajectoryDuration(      _paramHelper, PARAM_ID_TRAJ_TIME_HAND);
    _tasks.posture.linkParameterTrajectoryDuration(        _paramHelper, PARAM_ID_TRAJ_TIME_POSTURE);
    
    // link module input streaming parameters to member variables
    _tasks.momentum.linkParameterComDes(         _paramHelper, PARAM_ID_XDES_COM);
    _tasks.supportForearm.linkParameterPoseDes(  _paramHelper, PARAM_ID_XDES_FOREARM);
    _tasks.graspHand.linkParameterPoseDes(       _paramHelper, PARAM_ID_XDES_HAND);
    _tasks.posture.linkParameterPostureDes(      _paramHelper, PARAM_ID_QDES);
    
#ifndef COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    YARP_ASSERT(_paramHelper->linkParam(PARAM_ID_H_W2B,   _H_w2b.data()));
#endif
    _tasks.momentum.linkParameterComRef(        _paramHelper, PARAM_ID_XREF_COM);
    _tasks.supportForearm.linkParameterPosRef(  _paramHelper, PARAM_ID_XREF_FOREARM);
    _tasks.graspHand.linkParameterPosRef(       _paramHelper, PARAM_ID_XREF_HAND);
    _tasks.posture.linkParameterPostureRef(     _paramHelper, PARAM_ID_QREF);
    
    _tasks.momentum.linkParameterCom(               _paramHelper, PARAM_ID_X_COM);
    _tasks.momentum.linkParameterMomentum(          _paramHelper, PARAM_ID_MOMENTUM);
    _tasks.momentum.linkParameterMomentumIntegral(  _paramHelper, PARAM_ID_MOMENTUM_INTEGRAL);
    _tasks.supportForearm.linkParameterPose(        _paramHelper, PARAM_ID_X_FOREARM);
    _tasks.graspHand.linkParameterPose(             _paramHelper, PARAM_ID_X_HAND);
    YARP_ASSERT(_paramHelper->linkParam(            PARAM_ID_Q,   _qjDeg.data()));
    
    _tasks.momentum.linkParameterComVel(    _paramHelper,           PARAM_ID_DX_COM);
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_X_BASE,        _robotState.xBase.p));
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_V_BASE,        _robotState.vBase.data()));
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_JOINT_TORQUES, _robotState.torques.data()));
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_DQ,            _dqjDeg.data()));

    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_FORCE_FRICTION,    &_forceFriction));
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_MOMENT_FRICTION,   &_momentFriction));
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_KP_CONSTRAINTS,    _kpConstraints.data()));
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_WRENCH_WEIGHTS,    _wrenchWeights.data()));
    
    _tasks.leftFoot.linkParameterForceInequalities(             _paramHelper, PARAM_ID_FORCE_INEQ_L_FOOT);
    _tasks.rightFoot.linkParameterForceInequalities(            _paramHelper, PARAM_ID_FORCE_INEQ_R_FOOT);
    _tasks.supportForearmConstr.linkParameterForceInequalities( _paramHelper, PARAM_ID_FORCE_INEQ_FOREARM);
    
    _tasks.leftFoot.setMaxNormalForce(              FORCE_NORMAL_MAX);
    _tasks.rightFoot.setMaxNormalForce(             FORCE_NORMAL_MAX);
    _tasks.supportForearmConstr.setMaxNormalForce(  FORCE_NORMAL_MAX);
    _tasks.leftFoot.setMinNormalForce(              FORCE_NORMAL_MIN);
    _tasks.rightFoot.setMinNormalForce(             FORCE_NORMAL_MIN);
    _tasks.supportForearmConstr.setMinNormalForce(  FORCE_NORMAL_MIN);
    
    // JOINT LIMIT TASK
    _tasks.jointLimits.setTimestep(50*getRate()*1e-3);
    _tasks.jointLimits.linkParameterJointLimitMinimumDistance(_paramHelper, PARAM_ID_JNT_LIM_MIN_DIST);
    _tasks.jointLimits.linkParameterQmax(_paramHelper, PARAM_ID_Q_MAX);
    _tasks.jointLimits.linkParameterQmin(_paramHelper, PARAM_ID_Q_MIN);
    _tasks.jointLimits.linkParameterDQmax(_paramHelper, PARAM_ID_DQ_MAX);
    _tasks.jointLimits.linkParameterDDQmax(_paramHelper, PARAM_ID_DDQ_MAX);
    _tasks.jointLimits.linkParameterQnormalized(_paramHelper, PARAM_ID_NORMALIZED_Q);
    
    // PLANNING PARAMETERS
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_GO_DOWN_COM,   _goDown_com.data()));
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_GO_DOWN_HAND,  _goDown_hand.data()));
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_GO_DOWN_Q,     _goDown_q.data()));
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_GO_UP_COM,     _goUp_com.data()));
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_GO_UP_HAND,    _goUp_hand.data()));
    YARP_ASSERT(_paramHelper->linkParam(    PARAM_ID_GO_UP_Q,       _goUp_q.data()));

    // Register callbacks for some module commands and parameters
    YARP_ASSERT(_paramHelper->registerParamValueChangedCallback(PARAM_ID_FORCE_FRICTION,    this));
    YARP_ASSERT(_paramHelper->registerParamValueChangedCallback(PARAM_ID_MOMENT_FRICTION,   this));
    YARP_ASSERT(_paramHelper->registerParamValueChangedCallback(PARAM_ID_KP_CONSTRAINTS,    this));
    YARP_ASSERT(_paramHelper->registerParamValueChangedCallback(PARAM_ID_WRENCH_WEIGHTS,    this));
    
    parameterUpdated(_paramHelper->getParamProxy(PARAM_ID_FORCE_FRICTION));
    parameterUpdated(_paramHelper->getParamProxy(PARAM_ID_MOMENT_FRICTION));
    parameterUpdated(_paramHelper->getParamProxy(PARAM_ID_KP_CONSTRAINTS));
    parameterUpdated(_paramHelper->getParamProxy(PARAM_ID_WRENCH_WEIGHTS));
    
    YARP_ASSERT(_paramHelper->linkParam(PARAM_ID_INTEGRATE_EOM, &_integrateEoM));
    _integrator.linkParameterToVariable(ConstrainedDynamicsIntegrator::TIMESTEP,        _paramHelper, PARAM_ID_INTEGRATOR_DT);
    _integrator.linkParameterToVariable(ConstrainedDynamicsIntegrator::CONSTR_NUM_DAMP, _paramHelper, PARAM_ID_INTEGRATOR_DAMP);
    YARP_ASSERT(_integrator.addConstraints(LEFT_FOOT_LINK_NAME));
    YARP_ASSERT(_integrator.addConstraints(RIGHT_FOOT_LINK_NAME));
    
    YARP_ASSERT(_paramHelper->registerCommandCallback(COMMAND_ID_START,           this));
    YARP_ASSERT(_paramHelper->registerCommandCallback(COMMAND_ID_STOP,            this));
    YARP_ASSERT(_paramHelper->registerCommandCallback(COMMAND_ID_GO_DOWN,         this));
    YARP_ASSERT(_paramHelper->registerCommandCallback(COMMAND_ID_GO_UP,           this));

#ifdef COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    _Ha.R = Rotation(0,0,1, 0,-1,0, 1,0,0);   // rotation to align foot Z axis with gravity, Ha=[0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1]
#endif

    // read _robot status (to be done before initializing trajectory generators)
#ifdef DO_NOT_USE_WHOLE_BODY_STATE_INTERFACE
    _sensors = new icubWholeBodySensors((_name+"_sensor").c_str(), _robotName.c_str());
    _sensors->addSensors(SENSOR_ENCODER, ICUB_MAIN_JOINTS);
    _sensors->addSensors(SENSOR_TORQUE,  ICUB_MAIN_JOINTS);
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
    if(!_contactReader.init(WHOLE_BODY_DYNAMICS_NAME))
        return false;
    _solver.init(_robotState);
    _solver.computeSolution(_robotState, _tauDes);
    
    printf("\n\n");
    return true;
}

//*************************************************************************************************************************
void WholeBodyReachThread::run()
{
    _time += getRate()*1e-3;
    
    _paramHelper->lock();
    _paramHelper->readStreamParams();

    readRobotStatus();                      // read encoders, compute positions and Jacobians
    
//    Frame H_left;
//    _robot->computeH(_robotState.qJ.data(), _robotState.xBase, LINK_ID_LEFT_FOOT, H_left);
//    sendMsg("Left foot pose:  "+toString(Vector3d::Map(H_left.p),4));
//    sendMsg("dq "+jointToString(WBR_RAD2DEG * _robotState.dqJ));
//    sendMsg("dq norm    "+toString(WBR_RAD2DEG * _robotState.dqJ.norm()));
//    sendMsg("x base\n"+_robotState.xBase.toString());
    bool res = _solver.computeSolution(_robotState, _tauDes);   // compute desired joint torques
    
    if(_status==WHOLE_BODY_REACH_ON)
    {
        if(areDesiredJointTorquesTooLarge())    // check desired joint torques are not too large
        {
            preStopOperations();            // stop the controller
            cout<<"\n***** ERROR: CONTROLLER STOPPED BECAUSE DESIRED JOINT TORQUES ARE TOO LARGE: "
                <<toString(_tauDes.transpose(),1)<<endl;
        }
        else if(res==false)
        {
            preStopOperations();            // stop the controller
            cout<<"\n***** ERROR: CONTROLLER STOPPED BECAUSE SOLVER COULDN'T FIND A SOLUTION\n";
        }
        else if(_integrateEoM)
        {
//            sendMsg("q PRE  "+jointToString(WBR_RAD2DEG * _robotState.qJ));
            int nt = getRate()*1e-3/_integrator._timestep;
            sendMsg("Number of calls to integrator: "+toString(nt));
            _robotState.xBase   = _robotStateIntegrator.xBase;
            _robotState.qJ      = _robotStateIntegrator.qJ;
            _robotState.dq      = _robotStateIntegrator.dq;
            for(int i=0; i<nt; i++)
            {
                _integrator.integrate(_tauDes,
                                      _robotState.xBase,    _robotState.qJ,    _robotState.dq,
                                      _robotStateIntegrator.xBase, _robotStateIntegrator.qJ, _robotStateIntegrator.dq);
                
                if(i==0)
                {
                    sendMsg("|| ddqDes - ddqIntegrator || = "+toString((_integrator._ddq_first_call - _solver._ddqDes).norm()));
//#define DEBUG_FORWARD_DYNAMICS
#ifdef DEBUG_FORWARD_DYNAMICS
                    // Null-space basis may be different but span the same space.
                    // Given two different basis Z1 and Z2 of the same space, this needs to hold:
                    // Z2 = Z1 * Z1^T * Z2
                    // Z1 = Z2 * Z2^T * Z1
                    const MatrixRXd& Z1 = _integrator._Z;
                    const MatrixRXd& Z2 = _solver._Zc;
                    double Z1err = (Z1 - Z2*Z2.transpose()*Z1).norm();
                    double Z2err = (Z2 - Z1*Z1.transpose()*Z2).norm();
                    double ZMZerr = (_integrator._ZMZ - _solver._ZMZ).norm();
                    VectorXd rhs_solver = (_solver._tau_np6     - _solver._h     - _solver._M    *_solver._ddqBar);
                    VectorXd rhs_integr = (_integrator._tau_np6 - _integrator._h - _integrator._M*_integrator._ddqBar);
                    double MddqBar_err = (_solver._M*_solver._ddqBar - _integrator._M*_integrator._ddqBar).norm();
                    double ddqFD_err = (_integrator._ddq_first_call - _solver._ddqFD).norm();
                    
                    sendMsg("Jc err    "+toString((_integrator._A - _solver._Jc).norm()));
                    sendMsg("dJc*dq err "+toString((_integrator._b + _solver._dJcdq).norm())); // b = -dJc*dq
                    sendMsg("damp err "+toString(_integrator._numericalDamping - _solver._numericalDampingConstr));
                    sendMsg("Z1 err    "+toString(Z1err));
                    sendMsg("Z2 err    "+toString(Z2err));
//                  sendMsg("Z1\n"+toString(Z1.transpose(),1,"\n",16));
//                  sendMsg("Z2\n"+toString(Z2.transpose(),1,"\n",16));
//                    sendMsg("ZMZ err   "+toString(ZMZerr));
                    sendMsg("rhs err   "+toString((rhs_solver-rhs_integr).norm()));
//                    sendMsg("tau err   "+toString((_solver._tau_np6-_integrator._tau_np6).norm()));
//                    sendMsg("qJ err    "+toString((_solver._qj - _integrator._qj).norm()));
                    sendMsg("xB err<1e-10? "+toString(isEqual(_solver._xB, _integrator._xB_i, 1e-10)));
//                    sendMsg("xB solver     "+_solver._xB.toString());
//                    sendMsg("xB integrator "+_integrator._xB.toString());
                    sendMsg("dq err            "+toString((_solver._dq - _integrator._dq).norm()));
//                    sendMsg("h err         "+toString((_solver._h-_integrator._h).norm()));
//                    sendMsg("M err         "+toString((_solver._M - _integrator._M).norm()));
//                    sendMsg("ddqBar err    "+toString((_solver._ddqBar    -_integrator._ddqBar).norm()));
//                    sendMsg("M*ddqBar err "+toString(MddqBar_err));
//                    sendMsg("ddq_c err     "+toString((_integrator._ddq_c - _solver._ddq_c).norm()));
                    sendMsg("|| ddqFD - ddqIntegrator || = "+toString(ddqFD_err));
#endif
                }
                _robotState.xBase   = _robotStateIntegrator.xBase;
                _robotState.qJ      = _robotStateIntegrator.qJ;
                _robotState.dq      = _robotStateIntegrator.dq;
            }
//            sendMsg("q POST "+jointToString(WBR_RAD2DEG * _robotState.qJ));
            
            
            // copy base and joint velocities into _robotState.dqJ and _robotState.vBase
            _robotState.vBase   = _robotState.dq.head<6>();
            _robotState.dqJ     = _robotState.dq.tail(_n);

            // send the current state as desired state for the simulator/real robot
            _robot->setControlReference(_robotState.qJ.data());
        }
        else
        {
            _robot->setControlReference(_tauDes.data());
        }
    }

    _paramHelper->sendStreamParams();
    _paramHelper->unlock();
    sendMsg("");
    getLogger().countdown();
}

//*************************************************************************************************************************
bool WholeBodyReachThread::readRobotStatus(bool blockingRead)
{
    bool res = true;
    
    if(!_integrateEoM)
    {
        // temporary replacement of _robot->getEstimate because it's too slow
#ifdef DO_NOT_USE_WHOLE_BODY_STATE_INTERFACE
        res = res && _sensors->readSensors(SENSOR_ENCODER, _robotState.qJ.data(),      _qJStamps.data(), blockingRead);
        //    res = res && _sensors->readSensors(SENSOR_TORQUE,  _robotState.torques.data(), NULL,             blockingRead);
#else
        res = res && _robot->getEstimates(ESTIMATE_JOINT_POS,    _robotState.qJ.data(),     -1.0, blockingRead);
        res = res && _robot->getEstimates(ESTIMATE_JOINT_VEL,    _robotState.dqJ.data(),    -1.0, blockingRead);
#endif
    }

#ifdef DO_NOT_USE_WHOLE_BODY_STATE_INTERFACE
    for(int i=0; i<_n; i++)
        _qJ_yarp(i) = _robotState.qJ(i);
    AWPolyElement el;
    el.data = _qJ_yarp;
    if(_integrateEoM)
        el.time = _time;        // Use integrator time
    else
        el.time = Time::now();  // Use yarp time rather than _qJStamps to be synchronized with simulator
    
    //    static double _qJStampsOld = 0.0;
    //    static double timeStampOld = 0.0;
    //    double timeStamp = Time::now();
    //    sendMsg("q stamp-stampOld = "+toString(_qJStamps[0]-_qJStampsOld));
    //    sendMsg("t stamp-stampOld = "+toString(timeStamp-timeStampOld)+ "\t Is system clock? "+toString(Time::isSystemClock()));
    //    _qJStampsOld = _qJStamps[0];
    //    timeStampOld = timeStamp;
    
    _dqJ_yarp = _dqFilt->estimate(el);
    if(!_integrateEoM)          // if integrating EoM use integrator dq
        for(int i=0; i<_n; i++)
            _robotState.dqJ(i) = _dqJ_yarp(i);
#endif
    
#ifdef COMPUTE_WORLD_2_BASE_ROTOTRANSLATION
    // compute base pose assuming left foot is fixed on the ground
    _robot->computeH(_robotState.qJ.data(), Frame(), LINK_ID_LEFT_FOOT, _H_base_leftFoot);
    _H_base_leftFoot = _H_base_leftFoot*_Ha;
    _H_base_leftFoot.setToInverse().get4x4Matrix(_H_w2b.data());    // homogeneous transformation from world (i.e. left foot) to base
#endif
    // base orientation conversion
    if(!_integrateEoM)
        _robotState.xBase.set4x4Matrix(_H_w2b.data());

    // compute Jacobians of both feet to estimate base velocity
    res = res && _robot->computeJacobian(_robotState.qJ.data(), _robotState.xBase, LINK_ID_RIGHT_FOOT,  _JfootR.data());
    res = res && _robot->computeJacobian(_robotState.qJ.data(), _robotState.xBase, LINK_ID_LEFT_FOOT,   _JfootL.data());
    _Jc.topRows<6>()     = _JfootR;
    _Jc.bottomRows<6>()  = _JfootL;
    
    // estimate base velocity from joint velocities and constraint Jacobian Jc
    _svdJcb.compute(_Jc.leftCols<6>(), ComputeThinU | ComputeThinV);
    if(!_integrateEoM)
        _robotState.vBase = _svdJcb.solve(-_Jc.rightCols(_n)*_robotState.dqJ);

    // copy base and joint velocities into _robotState.dq
    _robotState.dq.head<6>()    = _robotState.vBase;
    _robotState.dq.tail(_n)     = _robotState.dqJ;
    
    // convert joint angles and velocities from rad to deg
    _qjDeg  = CTRL_RAD2DEG*_robotState.qJ;
    _dqjDeg = CTRL_RAD2DEG*_robotState.dqJ;
    
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
bool WholeBodyReachThread::preStartOperations()
{
    // no need to lock because the mutex is already locked
    bool res = readRobotStatus(true);
    // initialize trajectory generators
    _solver.init(_robotState);

#ifdef DEBUG_INIT
    _tasks.momentum.update(_robotState);
    _tasks.posture.update(_robotState);
    cout<<"INIT Desired Momentum:    "<<toString(_tasks.momentum.getEqualityVector(),2)<<endl;
    cout<<"INIT Desired ddq posture: "<<toString(_tasks.posture.getEqualityVector(),2)<<endl;
    _solver.init(_robotState);
#endif
    
    if(_integrateEoM==false)
        res = res && _robot->setControlMode(CTRL_MODE_TORQUE, _tauDes.data());
    else
    {
        /// initialize integrator state
        _robotStateIntegrator.xBase   = _robotState.xBase;
        _robotStateIntegrator.qJ      = _robotState.qJ;
        _robotStateIntegrator.dq      = _robotState.dq;
    }
    
    if(res)
        _status = WHOLE_BODY_REACH_ON;                 // set thread status to "on"
    else
    {
        _robot->setControlMode(CTRL_MODE_POS);
        cout<<"\nError in WholeBodyReachThread::preStartOperations(), controller is not gonna start!\n\n";
    }
    return res;
}

//*************************************************************************************************************************
void WholeBodyReachThread::preStopOperations()
{
    // no need to lock because the mutex is already locked
    _robot->setControlMode(CTRL_MODE_POS);              // set position control mode
    _status = WHOLE_BODY_REACH_OFF;                     // set thread status to "off"
}

//*************************************************************************************************************************
void WholeBodyReachThread::threadRelease()
{
}

//*************************************************************************************************************************
void WholeBodyReachThread::parameterUpdated(const ParamProxyInterface *pd)
{
    switch(pd->id)
    {
    case PARAM_ID_FORCE_FRICTION:
        _tasks.leftFoot.setForceFrictionCoefficient(_forceFriction);
        _tasks.rightFoot.setForceFrictionCoefficient(_forceFriction);
        _tasks.supportForearmConstr.setForceFrictionCoefficient(_forceFriction);
        break;
    case PARAM_ID_MOMENT_FRICTION:
        _tasks.leftFoot.setMomentFrictionCoefficient(_momentFriction);
        _tasks.rightFoot.setMomentFrictionCoefficient(_momentFriction);
        break;
    case PARAM_ID_KP_CONSTRAINTS:
        _tasks.leftFoot.setProportionalGain(_kpConstraints);
        _tasks.rightFoot.setProportionalGain(_kpConstraints);
        _tasks.supportForearmConstr.setProportionalGain(_kpConstraints.head<3>());
        break;
    case PARAM_ID_WRENCH_WEIGHTS:
        _tasks.leftFoot.setWeights(_wrenchWeights);
        _tasks.rightFoot.setWeights(_wrenchWeights);
        _tasks.supportForearmConstr.setWeights(_wrenchWeights.head<3>());
        break;
    default:
        sendMsg("A callback is registered but not managed for the parameter "+pd->name, MSG_ERROR);
    }
}

//*************************************************************************************************************************
void WholeBodyReachThread::commandReceived(const CommandDescription &cd, const Bottle &params, Bottle &reply)
{
    cout<<"\ncommand received called\n\n";
    switch(cd.id)
    {
    case COMMAND_ID_START:
        preStartOperations();
        break;
    case COMMAND_ID_STOP:
        preStopOperations();
        break;
    case COMMAND_ID_GO_DOWN:
        {
            _tasks.graspHand.setPosDes(_goDown_hand);
            _tasks.momentum.setComDes(_goDown_com);
            _tasks.posture.setPostureDes(_goDown_q);
            break;
        }
    case COMMAND_ID_GO_UP:
        {
            _tasks.graspHand.setPosDes(_goUp_hand);
            _tasks.momentum.setComDes(_goUp_com);
            _tasks.posture.setPostureDes(_goUp_q);
            break;
        }
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
