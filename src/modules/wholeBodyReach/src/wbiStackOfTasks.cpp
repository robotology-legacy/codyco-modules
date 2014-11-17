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

#include <wholeBodyReach/wbiStackOfTasks.h>
#include <wholeBodyReach/Stopwatch.h>
#include <Eigen/SVD>

#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/math.h>
#include <iostream>
#include <cstdio>

using namespace wholeBodyReach;
using namespace paramHelp;
using namespace Eigen;
using namespace std;
using namespace iCub::ctrl;
using namespace yarp::math;


#define Jc_j        _Jc.rightCols(_n)
#define Jc_b        _Jc.leftCols<6>()
#define M_b         _M.topLeftCorner<6, 6>()
#define M_bj        _M.topRightCorner(6,_n)
#define M_u         _M.topRows<6>()
#define M_a         _M.bottomRows(_n)
#define h_b         _h.head<6>()
#define h_j         _h.tail(_n)
#define ddqDes_b    _ddqDes.head<6>()
#define ddqDes_j    _ddqDes.tail(_n)

#define PROFILE_WHOLE_SOLVER            "Whole solver"
#define PROFILE_DYNAMICS_COMPUTATION    "Dynamics computation"
#define PROFILE_FORCE_QP_PREP           "Force QP preparation"
#define PROFILE_FORCE_QP_MOMENTUM       "Force QP momentum task update"
#define PROFILE_FORCE_QP                "Force QP"
#define PROFILE_FORCE_TOTAL             "Force overall computation"
#define PROFILE_DDQ_DYNAMICS_CONSTR     "Compute dynamics-consistent ddq"
#define PROFILE_DDQ_CONTACT_CONSTR      "Compute contact-consistent ddq"
#define PROFILE_DDQ_POSTURE_TASK        "Compute posture task"

//**************************************************************************************************
wbiStackOfTasks::wbiStackOfTasks(wbi::wholeBodyInterface* robot, bool useNullspaceBase)
:   _robot(robot),
    _momentumTask(NULL),
    _postureTask(NULL),
    _jointLimitTask(NULL),
    _useNullspaceBase(useNullspaceBase),
    _ctrlAlg(WBR_CTRL_ALG_MOMENTUM_SOT)
{
    this->useNullspaceBase(_useNullspaceBase==1);
    
    _n = robot->getDoFs();
    _M.resize(_n+6,_n+6);
    _h.resize(_n+6);
    
    _Mb_inv.setZero();
    _Mb_llt = LLT<MatrixR6d>();
    _ddqDes.setZero(_n+6);
    _ddq_jDes.setZero(_n);
    _ddq_jPosture.setZero(_n);
    
    _A_i.setZero(6,_n+6);
    _a_i.setZero(6);

    resizeVariables();
}

//**************************************************************************************************
bool wbiStackOfTasks::computeSolution(RobotState& robotState, Eigen::VectorRef torques)
{
#ifdef DEBUG_FORWARD_DYNAMICS
    _qj = robotState.qJ;
    _xB = robotState.xBase;
    _dq = robotState.dq;
#endif
    START_PROFILING(PROFILE_WHOLE_SOLVER);
    
    assert(_momentumTask!=NULL);
    assert(_postureTask!=NULL);
    
    useNullspaceBase(_useNullspaceBase==1);
    
    //*********************************
    // COMPUTE ROBOT DYNAMICS
    //*********************************
    START_PROFILING(PROFILE_DYNAMICS_COMPUTATION);
    {
        _robot->computeGeneralizedBiasForces(robotState.qJ.data(), robotState.xBase,
                                             robotState.dqJ.data(), robotState.vBase.data(),
                                             robotState.g.data(), _h.data());
        _robot->computeMassMatrix(robotState.qJ.data(), robotState.xBase, _M.data());
    }
    STOP_PROFILING(PROFILE_DYNAMICS_COMPUTATION);
    
    //************************************************
    // COMPUTE QUANTITIES RELATED TO CONTACT FORCES
    //************************************************
    START_PROFILING(PROFILE_FORCE_QP_PREP);
    {
        int index_k = 0, k=0, index_in=0, in=0;
        double forceNormalTot = 0.0;
        for(list<ContactConstraint*>::iterator it=_constraints.begin(); it!=_constraints.end(); it++)
        {
            ContactConstraint& c = **it;
            c.update(robotState);
            
            k  = c.getSize();    // number of constraint forces
            in = c.getNumberOfInequalities();
            
            c.getInequalityMatrix( _B.block(index_in, index_k, in, k));     // CI = [CI, t.getInequalityMatrix()]
            c.getInequalityVector( _b.segment(index_in, in) );              //  b = [b; t.getInequalityVector()]
            c.getMomentumMapping(  _X.middleCols(index_k, k)  );            //  X = [X, t.getMomentumMapping()]
            c.getEqualityMatrix(   _Jc.middleRows(index_k, k) );            // Jc = [Jc; t.getEqualityMatrix()]
            c.getEqualityVector(   _dJcdq.segment(index_k, k) );            // dJc_dq = [dJc_dq; t.getEqualityVector()]
            c.getWeights(          _fWeights.segment(index_k,k));
            
            // if the normal force of this constraint (computed at the last step) is not zero
            // then divide the weights by the normal force
            if(_fcDes(index_k+2)!=0.0)
            {
                _fWeights.segment(index_k,k) /= _fcDes(index_k+2);
                forceNormalTot += _fcDes(index_k+2);
            }
            else
                sendMsg("Normal force of contact "+c.getName()+" was zero at the previous loop!", MSG_STREAM_ERROR);
            
//            sendMsg("CI block "+c.getName()+":\n"+toString(_qp_force.CI.block(index_in, index_k, in, k),1,"\n",12));
//            sendMsg("ci0: "+toString(_qp_force.ci0.segment(index_in, in),1));
            
            index_k += k;
            index_in += in;
        }
        _dJcdq *= -1.0;
        
        assert(index_k==_k);
//        sendMsg("X:\n"+toString(_X,1,"\n",12));
//        sendMsg("CI:\n"+toString(_qp_force.CI,1,"\n",12));
//        sendMsg("ci0: "+toString(_qp_force.ci0,1));
        
        START_PROFILING(PROFILE_FORCE_QP_MOMENTUM);
        {
            _momentumTask->update(robotState);
            _momentumTask->getEqualityVector(_momentumDes);
            if(forceNormalTot!=0.0)
                _fWeights *= forceNormalTot;
//            _fWeights.normalize();
        }
        STOP_PROFILING(PROFILE_FORCE_QP_MOMENTUM);
    }
    STOP_PROFILING(PROFILE_FORCE_QP_PREP);
    
    // update postural task
    _postureTask->update(robotState);
    _postureTask->getEqualityVector(_ddq_jPosture);
    
    bool res;
    switch(_ctrlAlg)
    {
        case WBR_CTRL_ALG_MOMENTUM_SOT:         res = computeMomentumSoT(robotState, torques);      break;
        case WBR_CTRL_ALG_NULLSPACE_PROJ:       res = computeNullspaceProj(robotState, torques);    break;
        case WBR_CTRL_ALG_COM_POSTURE:          res = computeComPosture(robotState, torques);       break;
        case WBR_CTRL_ALG_MOMENTUM_POSTURE:     res = computeMomentumPosture(robotState, torques);  break;
        case WBR_CTRL_ALG_MOMENTUM_SOT_SAFE:    res = computeMomentumSoT_safe(robotState, torques); break;
        case WBR_CTRL_ALG_COM_SOT:              res = computeComSoT(robotState, torques);           break;
        default: printf("WbiStackOfTask: ERROR unmanaged control algorithm!\n");
    }
    
//    sendMsg("tau des: "+jointToString(torques,1));
//    sendMsg("fc des: "+toString(_fcDes,1));
//    sendMsg("Jc^t*f: "+toString(_Jc.transpose()*_fcDes,1));
//    sendMsg("dJc*dq: "+toString(_dJcdq,2));
//    sendMsg("xB:    \n"+robotState.xBase.toString());
//    sendMsg("q:       "+jointToString(WBR_RAD2DEG*robotState.qJ,1));
//    sendMsg("dq:      "+jointToString(WBR_RAD2DEG*robotState.dq,1));
//    sendMsg("ddqDes:  "+jointToString(WBR_RAD2DEG*_ddqDes,2));
//    sendMsg("Torques measured: "+jointToString(robotState.torques));
    
    
    if(_ctrlAlg!=WBR_CTRL_ALG_NULLSPACE_PROJ)
    {
        int index_k = 0, k=0;
        for(list<ContactConstraint*>::iterator it=_constraints.begin(); it!=_constraints.end(); it++)
        {
            ContactConstraint& c = **it;
            k  = c.getSize();    // number of constraint forces
            c.setDesiredConstraintForce(_fcDes.segment(index_k, k));
            index_k += k;
        }
    }
    
    if(_jointLimitTask!=NULL)
        _jointLimitTask->setDdqDes(_ddqDes.tail(_n));
    
    STOP_PROFILING(PROFILE_WHOLE_SOLVER);
    
#ifdef DEBUG_FORWARD_DYNAMICS
    _ddqFD.resize(_n+6);   // accelerations computed by the forward dynamics algorithm
    constrainedForwardDynamics(robotState.g, torques, _xB, _qj, _dq, _ddqFD);
    sendMsg("ddqFD-ddqDes: "+toString((_ddqFD-_ddqDes).norm()));
#endif
    
    return res;
}

//**************************************************************************************************
bool wbiStackOfTasks::computeComSoT(RobotState& robotState, Eigen::VectorRef torques)
{
    sendMsg("ComputeComSoT");
    double tmp;
    int n_jl_in = _jointLimitTask->getNumberOfInequalities();
    int n_f_in  = _B.rows();
    
    bool jointLimitActive = _jointLimitTask!=NULL;
    if(jointLimitActive)
        _jointLimitTask->update(robotState);
    
    // Set-up equality constraints for QP
    computeMb_inverse();
    _Mb_inv_M_bj  = _Mb_inv * M_bj;
    _Mb_inv_J_cbT = _Mb_inv * Jc_b.transpose();
    
    _Jc_Sbar                    = Jc_j - (Jc_b * _Mb_inv_M_bj);
    _qp_force.CE.leftCols(_n)   = _Jc_Sbar;
    _qp_force.CE.rightCols(_k)  = Jc_b*_Mb_inv_J_cbT;
    _qp_force.ce0               = _dJcdq - Jc_b*(_Mb_inv*h_b);
    // Set-up inequality constraints for QP
    _jointLimitTask->getInequalityMatrix(_qp_force.CI.topLeftCorner(n_jl_in, _n));
    _jointLimitTask->getInequalityVector(_qp_force.ci0.head(n_jl_in));
    _qp_force.CI.bottomRightCorner(n_f_in, _k) = _B;
    _qp_force.ci0.tail(n_f_in)    = _b;
    // Set-up cost function for QP
    _qp_force.A.rightCols(_k)   = _X.topRows<3>();
    _qp_force.a                 = _momentumDes.head<3>();
    // set higher numerical damping for joint acc than for contact forces
    _qp_force.w.tail(_k)        = _numericalDampingTask*_fWeights;
    _qp_force.w.head(_n).setConstant(_numericalDampingTask*_numericalDampingTask);
//    sendMsg("w*1e6 = "+toString(_qp_force.w.transpose()*1e6));
    
    if(!solveForceQP(robotState))
        return false;
    
    _qp_force.computeQpNullSpace();
    _Z = _qp_force.N_CE;
    
//#define DEBUG_POSE_TASKS
    // Solve the equality motion task
    // N=I
    // for i=1:K
    //      A = A_i*Sbar*N
    //      ddq_j = A^+ (b_i - A_i*ddq)
    //      ddq  += Sbar*ddq_j
    //      N    -= A^+ A
    // end
    
    list<MinJerkPDLinkPoseTask*>::iterator it;
    for(it=_equalityTasks.begin(); it!=_equalityTasks.end(); it++)
    {
        MinJerkPDLinkPoseTask& t = **it;
        t.update(robotState);
        
        t.getEqualityMatrix(_A_i);
        t.getEqualityVector(_a_i);
        
        // even if CI is constant, we need to update it every time because we overwrite it
        _jointLimitTask->getInequalityMatrix(_qp_motion1.CI.topLeftCorner(n_jl_in, _n));
        _jointLimitTask->getInequalityVector(_qp_motion1.ci0.head(n_jl_in));
        _qp_motion1.ci0.head(n_jl_in)       += _qp_motion1.CI.topLeftCorner(n_jl_in, _n)*ddqDes_j;
        _qp_motion1.CI.bottomRightCorner(n_f_in, _k)    = _B;
        _qp_motion1.ci0.tail(n_f_in)                    = _b;
        _qp_motion1.ci0.tail(n_f_in)        += _B*_fcDes;
        // the following two lines are equivalent to (but more efficient than):
        // _qp_motion1.CI *= _Z;
        _qp_motion1.CI.topRows(n_jl_in)   = _qp_motion1.CI.topLeftCorner(n_jl_in, _n)*_Z.topRows(_n);
        _qp_motion1.CI.bottomRows(n_f_in) = _qp_motion1.CI.bottomRightCorner(n_f_in, _k)*_Z.bottomRows(_k);
        
        _qp_motion1.A.leftCols(_n)  = _A_i.rightCols(_n) - _A_i.leftCols<6>()*_Mb_inv_M_bj;
        _qp_motion1.A.rightCols(_k) = _A_i.leftCols<6>()*_Mb_inv_J_cbT;
        _qp_motion1.A       *= _Z;
        _qp_motion1.w       = _numericalDampingTask*VectorXd::Ones(_n+_k);
        _qp_motion1.a       = _a_i - _A_i*_ddqDes;
        
        for(int i=0; i<_qp_motion1.ci0.size(); i++)
            if(_qp_motion1.ci0(i)<-ZERO_NUM)
            {
                sendMsg(t.getName()+" ci0("+toString(i)+") = "+toString(_qp_motion1.ci0(i)), MSG_ERROR);
                return false;
            }
            else if(_qp_motion1.ci0(i)<0.0)
            {
                // The previous solution slightly violates the constraints
                // -ZERO_NUM < ci0(i) < 0
                sendMsg(t.getName()+" slight constraint violation, ci0("+toString(i)+") = "+
                        toString(_qp_motion1.ci0(i)), MSG_STREAM_WARNING);
                _qp_motion1.ci0(i) = 0.0;
            }
        
        
        double res = _qp_motion1.solveQP();
        _ddq_jDes  = _qp_motion1.x.head(_n);
        
        if(res == std::numeric_limits<double>::infinity())
        {
            sendMsg(t.getName()+" QP could not be solved!", MSG_ERROR);
            sendMsg(_qp_motion1.toString(), MSG_DEBUG);
            return false;
        }
        if(_qp_motion1.activeSetSize>0)
            sendMsg(t.getName()+". Active joint limits: "+
                    toString(_qp_motion1.activeSet.head(_qp_motion1.activeSetSize).transpose()));
        if((tmp=(_qp_motion1.x-_Z*_qp_motion1.x).norm()) > ZERO_NUM)
            sendMsg(t.getName()+". Part of solution outside the null space of previous tasks: "+
                    toString(tmp), MSG_STREAM_WARNING);

        ddqDes_b    -= _Mb_inv_M_bj*_ddq_jDes;
        ddqDes_b    += _Mb_inv_J_cbT*_qp_motion1.x.tail(_k);
        ddqDes_j    += _ddq_jDes;
        _fcDes      += _qp_motion1.x.tail(_k);
        
        _qp_motion1.computeQpNullSpace();
        _Z *= _qp_motion1.N_CE;

#ifdef  DEBUG_POSE_TASKS
        _jointLimitTask->setDdqDes(ddqDes_j);
        //        sendMsg(t.getName()+" Jacobian =\n"+toString(_A_i));
        //        int nzsv = _A_svd.nonzeroSingularValues();
        //        sendMsg(t.getName()+". "+toString(nzsv)+"-th sing val = "+toString(_A_svd.singularValues()(nzsv-1)));
        sendMsg(t.getName()+". Contact constr error = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
        sendMsg(t.getName()+" ||A*x-b|| =    "+toString((_A_i*_ddqDes-_a_i).norm()));
#endif
    }
    
    START_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    {
        // even if CI is constant, we need to update it every time because we overwrite it
        _jointLimitTask->getInequalityMatrix(_qp_posture.CI.topLeftCorner(n_jl_in, _n));
        _jointLimitTask->getInequalityVector(_qp_posture.ci0.head(n_jl_in));
        _qp_posture.ci0.head(n_jl_in)       += _qp_posture.CI.topLeftCorner(n_jl_in, _n)*ddqDes_j;
        _qp_posture.CI.bottomRightCorner(n_f_in, _k)    = _B;
        _qp_posture.ci0.tail(n_f_in)                    = _b;
        _qp_posture.ci0.tail(n_f_in)                    += _B*_fcDes;
        // The following two lines are equivalent to:
        // _qp_posture.CI *= _Z;
        // but they are more efficient and we can spare setting to zero the empty parts of CI
        _qp_posture.CI.topRows(n_jl_in)   = _qp_posture.CI.topLeftCorner(n_jl_in, _n)*_Z.topRows(_n);
        _qp_posture.CI.bottomRows(n_f_in) = _qp_posture.CI.bottomRightCorner(n_f_in, _k)*_Z.bottomRows(_k);
        
        _qp_posture.A       = _Z.topRows(_n);
        _qp_posture.w       = _numericalDampingTask*VectorXd::Ones(_n+_k);
        _qp_posture.a       = _ddq_jPosture - ddqDes_j;
        
        for(int i=0; i<_qp_posture.ci0.size(); i++)
            if(_qp_posture.ci0(i) < -ZERO_NUM)
            {
                sendMsg("ERROR: Posture task, previous QP solution violates inequality "+toString(i)+
                        " by "+toString(_qp_posture.ci0(i)), MSG_ERROR);
                return false;
            }
            else if(_qp_posture.ci0(i)<0.0)
            {
                // The previous solution slightly violates the constraints: -ZERO_NUM < ci0(i) < 0
                sendMsg("Posture slight constraint violation, ci0("+toString(i)+") = "+
                        toString(_qp_posture.ci0(i)), MSG_STREAM_WARNING);
                _qp_posture.ci0(i) = 0.0;
            }
        
        double res = _qp_posture.solveQP(_numericalDampingConstr);
        _ddq_jDes  = _qp_posture.x.head(_n);
        
        if(res == std::numeric_limits<double>::infinity())
        {
            sendMsg("Posture QP could not be solved!", MSG_ERROR);
            sendMsg("Posture QP:\n"+_qp_posture.toString(), MSG_DEBUG);
            return false;
        }
        if(_qp_posture.activeSetSize>0)
            sendMsg("Posture task. Active joint limits: "+toString(_qp_posture.activeSet.head(_qp_posture.activeSetSize).transpose()));
        if((tmp=(_qp_posture.x-_Z*_qp_posture.x).norm())>ZERO_NUM)
            sendMsg("Posture task. Part of ddq_jDes outside the null space of previous tasks: "+
                    toString(tmp), MSG_STREAM_WARNING);
        
        ddqDes_b    -= _Mb_inv_M_bj*_ddq_jDes;
        ddqDes_b    += _Mb_inv_J_cbT*_qp_posture.x.tail(_k);
        ddqDes_j    += _ddq_jDes;
        _fcDes      += _qp_posture.x.tail(_k);
    }
    STOP_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    
    torques     = M_a*_ddqDes + h_j - Jc_j.transpose()*_fcDes;
    
    sendMsg("Linear momentum error = "+toString((_X*_fcDes-_momentumDes).head<3>().norm()));
    sendMsg("Base dynamics error   = "+toString((M_u*_ddqDes+h_b-Jc_b.transpose()*_fcDes).norm()));
    sendMsg("Contact constr error  = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
    
    
#ifdef  DEBUG_POSE_TASKS
    for(it=_equalityTasks.begin(); it!=_equalityTasks.end(); it++)
    {
        MinJerkPDLinkPoseTask& t = **it;
        t.getEqualityMatrix(_A_i);
        t.getEqualityVector(_a_i);
        sendMsg(t.getName()+" ||A*x-b|| =    "+toString((_A_i*_ddqDes-_a_i).norm()));
    }
#endif
    
    return true;
}

//**************************************************************************************************
bool wbiStackOfTasks::computeMomentumSoT_safe(RobotState& robotState, Eigen::VectorRef torques)
{
    sendMsg("ComputeMomentumSoT_safe");
    
    bool jointLimitActive = _jointLimitTask!=NULL;
    if(jointLimitActive)
        _jointLimitTask->update(robotState);
    
    // Set-up equality constraints for QP
    computeMb_inverse();
    _Mb_inv_M_bj= _Mb_inv * M_bj;
    _Jc_Sbar    = Jc_j - (Jc_b * _Mb_inv_M_bj);
    _qp_force.CE.leftCols(_n)   = _Jc_Sbar;
    _qp_force.CE.rightCols(_k)  = Jc_b*_Mb_inv*Jc_b.transpose();
    _qp_force.ce0               = _dJcdq - Jc_b*(_Mb_inv*h_b);
    // Set-up inequality constraints for QP
    int n_in = _jointLimitTask->getNumberOfInequalities();
    _jointLimitTask->getInequalityMatrix(_qp_force.CI.topLeftCorner(n_in, _n));
    _jointLimitTask->getInequalityVector(_qp_force.ci0.head(n_in));
    n_in = _B.rows();
    _qp_force.CI.bottomRightCorner(n_in, _k) = _B;
    _qp_force.ci0.tail(n_in)    = _b;
    // Set-up cost function for QP
    _qp_force.A.rightCols(_k)   = _X;
    _qp_force.a                 = _momentumDes;
    _qp_force.w.tail(_k)        = _numericalDampingTask*_fWeights;
    _qp_force.w.head(_n).setConstant(_numericalDampingDyn);
    
    if(!solveForceQP(robotState))
        return false;
    
    sendMsg("Momentum error  = "+toString((_X*_fcDes-_momentumDes).norm()));
//    sendMsg("Base dynamics error  = "+toString((M_u*_ddqDes+h_b-Jc_b.transpose()*_fcDes).norm()));
//    sendMsg("Contact constr error = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
    
    
#define DEBUG_POSE_TASKS
    // Solve the equality motion task
    // N=I
    // for i=1:K
    //      A = A_i*Sbar*N
    //      ddq_j = A^+ (b_i - A_i*ddq)
    //      ddq  += Sbar*ddq_j
    //      N    -= A^+ A
    // end
    
    _Z.setIdentity(_n,_n);
    
    // set the equality constraints for the first QP
    QpData *firstTask;
    if(_equalityTasks.empty())
        firstTask = &_qp_posture;
    else
        firstTask = &_qp_motion1;
    
    firstTask->CE  = _Jc_Sbar;
    // to ensure consistency with the contact forces found by the first QP
    // I make sure the contact-constraint residual is the same
    firstTask->ce0.setZero();
//    firstTask->ce0 = _dJcdq + _Jc*_ddqDes;
    
    double tmp;
    list<MinJerkPDLinkPoseTask*>::iterator it;
    for(it=_equalityTasks.begin(); it!=_equalityTasks.end(); it++)
    {
        MinJerkPDLinkPoseTask& t = **it;
        t.update(robotState);
        
        t.getEqualityMatrix(_A_i);
        t.getEqualityVector(_a_i);
        
        // even if CI is constant, we need to update it every time because we overwrite it
        _jointLimitTask->getInequalityMatrix(_qp_motion1.CI);
        _jointLimitTask->getInequalityVector(_qp_motion1.ci0);
        _qp_motion1.ci0     += _qp_motion1.CI*ddqDes_j;
        _qp_motion1.CI      *= _Z;
        
        _qp_motion1.A       = _A_i.rightCols(_n) - _A_i.leftCols<6>()*_Mb_inv_M_bj;
        _qp_motion1.A       *= _Z;
        _qp_motion1.w       = _numericalDampingTask*VectorXd::Ones(_n);
        _qp_motion1.a       = _a_i - _A_i*_ddqDes;
        for(int i=0; i<_qp_motion1.ci0.size(); i++)
            if(_qp_motion1.ci0(i)<-ZERO_NUM)
            {
                cout<<t.getName()+" ci0("+toString(i)+") = "+toString(_qp_motion1.ci0(i))<<endl;
                return false;
            }
            else if(_qp_motion1.ci0(i)<0.0)
            {
                // The previous solution slightly violates the constraints
                // -ZERO_NUM < ci0(i) < 0
                sendMsg(t.getName()+" slight constraint violation, ci0("+toString(i)+") = "+toString(_qp_motion1.ci0(i)));
                _qp_motion1.ci0(i) = 0.0;
            }
            
        
        double res = _qp_motion1.solveQP(_numericalDampingConstr);
        _ddq_jDes  = _qp_motion1.x;
        if(res == std::numeric_limits<double>::infinity())
        {
            cout<<t.getName()<<" QP could not be solved:\n"<<_qp_motion1.toString()<<endl;
            return false;
        }
        if(_qp_motion1.activeSetSize>0)
            sendMsg(t.getName()+". Active joint limits: "+toString(_qp_motion1.activeSet.head(_qp_motion1.activeSetSize).transpose()));

        if((tmp=(_ddq_jDes-_Z*_ddq_jDes).norm())>1e-10)
            sendMsg(t.getName()+". Part of ddq_jDes outside the null space of previous tasks: "+toString(tmp));
        
        ddqDes_b    -= _Mb_inv_M_bj*_ddq_jDes;
        ddqDes_j    += _ddq_jDes;
        
        _qp_motion1.computeQpNullSpace();
        _Z *= _qp_motion1.N_CE;
        
#ifdef  DEBUG_POSE_TASKS
//        _jointLimitTask->setDdqDes(ddqDes_j);
        //        sendMsg(t.getName()+" Jacobian =\n"+toString(_A_i));
        //        int nzsv = _A_svd.nonzeroSingularValues();
        //        sendMsg(t.getName()+". "+toString(nzsv)+"-th sing val = "+toString(_A_svd.singularValues()(nzsv-1)));
//        sendMsg(t.getName()+". Contact constr error = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
//        sendMsg(t.getName()+" ||A*x-b|| =    "+toString((_A_i*_ddqDes-_a_i).norm()));
#endif
    }
    
    //    sendMsg("Left arm ddq pre posture task:  "+toString(ddqDes_j.segment(3,5).transpose()));
    START_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    {
        _jointLimitTask->getInequalityMatrix(_qp_posture.CI);
        _jointLimitTask->getInequalityVector(_qp_posture.ci0);
        _qp_posture.ci0     += _qp_posture.CI*ddqDes_j;
        _qp_posture.CI      *= _Z;
        
        _qp_posture.A       = _Z;
        _qp_posture.w       = _numericalDampingTask*VectorXd::Ones(_n);
        _qp_posture.a       = _ddq_jPosture - ddqDes_j;
        for(int i=0; i<_qp_posture.ci0.size(); i++)
            if(_qp_posture.ci0(i) < -ZERO_NUM)
            {
                cout<<"Posture ci0("+toString(i)+") = "+toString(_qp_posture.ci0(i))<<endl;
                return false;
            }
            else if(_qp_posture.ci0(i)<0.0)
            {
                // The previous solution slightly violates the constraints
                // -ZERO_NUM < ci0(i) < 0
                sendMsg("Posture slight constraint violation, ci0("+toString(i)+") = "+toString(_qp_posture.ci0(i)));
                _qp_posture.ci0(i) = 0.0;
            }
        
        double res = _qp_posture.solveQP(_numericalDampingConstr);
        _ddq_jDes  = _qp_posture.x;
        
        if(res == std::numeric_limits<double>::infinity())
        {
            cout<<"Posture QP could not be solved:\n"<<_qp_posture.toString()<<endl;
            return false;
        }
        if(_qp_posture.activeSetSize>0)
            sendMsg("Posture task. Active joint limits: "+toString(_qp_posture.activeSet.head(_qp_posture.activeSetSize).transpose()));
        if((tmp=(_ddq_jDes-_Z*_ddq_jDes).norm())>1e-10)
            sendMsg("Posture task. Part of ddq_jDes outside the null space of previous tasks: "+toString(tmp));
        
        ddqDes_b    -= _Mb_inv_M_bj*_ddq_jDes;
        ddqDes_j    += _ddq_jDes;
    }
    STOP_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    
    torques     = M_a*_ddqDes + h_j - Jc_j.transpose()*_fcDes;
    
    //    sendMsg("fcDes              = "+toString(_fcDes,1));
    //    sendMsg("Left arm ddq post posture task: "+toString(ddqDes_j.segment(3,5).transpose()));
    //    sendMsg("Left arm ddq des posture task:      "+toString(_ddq_jPosture.segment(3,5).transpose()));
    //    sendMsg("ddq_jDes     = "+toString(_ddq_jDes,1));
    //    sendMsg("ddq_jPosture        = "+jointToString(_ddq_jPosture,1));
//    sendMsg("Base dynamics error  = "+toString((M_u*_ddqDes+h_b-Jc_b.transpose()*_fcDes).norm()));
    //    sendMsg("Joint dynamics error = "+toString((M_a*_ddqDes+h_j-Jc_j.transpose()*_fcDes-torques).norm()));
    sendMsg("Contact constr error = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
//    sendMsg("Contact constr residual = "+toString((_Jc*_ddqDes+_dJcdq).transpose()));
    
    
#ifdef  DEBUG_POSE_TASKS
    for(it=_equalityTasks.begin(); it!=_equalityTasks.end(); it++)
    {
        MinJerkPDLinkPoseTask& t = **it;
        t.getEqualityMatrix(_A_i);
        t.getEqualityVector(_a_i);
        sendMsg(t.getName()+" ||A*x-b|| =    "+toString((_A_i*_ddqDes-_a_i).norm()));
    }
#endif
    
    return true;
}

//**************************************************************************************************
bool wbiStackOfTasks::computeMomentumSoT(RobotState& robotState, Eigen::VectorRef torques)
{
    sendMsg("ComputeMomentumSoT");
    
    _qp_force.A     = _X;
    _qp_force.a     = _momentumDes;
    _qp_force.w     = _numericalDampingTask*_fWeights;
    _qp_force.CI    = _B;
    _qp_force.ci0   = _b;
    
    double res;
    START_PROFILING(PROFILE_FORCE_QP);
    {
        res     = _qp_force.solveQP();
        _fcDes  = _qp_force.x;
        if(res == std::numeric_limits<double>::infinity())
        {
            cout<<"Force QP could not be solved:\n"<<_qp_force.toString()<<endl;
            return false;
        }
        
        if(_qp_force.activeSetSize>0)
            sendMsg("Active constraints: "+toString(_qp_force.activeSet.head(_qp_force.activeSetSize).transpose()));
    }
    STOP_PROFILING(PROFILE_FORCE_QP);
    sendMsg("Momentum error  = "+toString((_X*_fcDes-_momentumDes).norm()));
    
    //*********************************
    // COMPUTE DESIRED ACCELERATIONS
    //*********************************
    
    // Compute ddq that respect dynamics:
    //     ddqBar = [Mb^{-1}*(Jc_u^T*f-h_u); zeros(n,1)];
    START_PROFILING(PROFILE_DDQ_DYNAMICS_CONSTR);
    {
        computeMb_inverse();
        ddqDes_b    = _Mb_inv*(Jc_b.transpose()*_fcDes - h_b);
        ddqDes_j.setZero();
    }
    STOP_PROFILING(PROFILE_DDQ_DYNAMICS_CONSTR);
    
    //    sendMsg("ddqDes (dynamic consistent) = "+toString(_ddqDes,1));
    //    sendMsg("Base dynamics error  = "+toString((M_u*_ddqDes+h_b-Jc_b.transpose()*_fcDes).norm()));
    
    // Compute ddq that respect also contact constraints:
    //      Sbar     = [-Mb^{-1}*Mbj; eye(n)];
    //      ddq_jDes = (Jc*Sbar).solve(-Jc*ddqDes - dJc_dq);
    //      ddqBar  += Sbar*ddq_jDes
    START_PROFILING(PROFILE_DDQ_CONTACT_CONSTR);
    {
        _Mb_inv_M_bj= _Mb_inv * M_bj;
        _Jc_Sbar    = Jc_j - (Jc_b * _Mb_inv_M_bj);
        _Jc_Sbar_svd.compute(_Jc_Sbar, _svdOptions);
        //        cout<<"Jc_b =\n"<< toString(Jc_b,1)<< endl;
        //        cout<<"Jc_b*ddqDes_b = "<< toString(Jc_b*ddqDes_b,1)<< endl;
        //        cout<<"ddqDes_b = "<< toString(ddqDes_b,1)<< endl;
        _ddq_jDes    = svdSolveWithDamping(_Jc_Sbar_svd, -_dJcdq-Jc_b*ddqDes_b, _numericalDampingConstr);
        //        _ddq_jDes    = _Jc_Sbar_svd.solve(-_dJcdq-Jc_b*ddqDes_b);
        ddqDes_b    -= _Mb_inv_M_bj*_ddq_jDes;
        ddqDes_j    += _ddq_jDes;
    }
    STOP_PROFILING(PROFILE_DDQ_CONTACT_CONSTR);
    
    //    sendMsg("ddqDes (contact consistent) = "+toString(_ddqDes,1));
    //    sendMsg("Base dynamics error  = "+toString((M_u*_ddqDes+h_b-Jc_b.transpose()*_fcDes).norm()));
//    sendMsg("Contact constr error = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
    
    // Now we can go on with the motion tasks, using the nullspace of dynamics and contacts:
    //      Z = nullspace(Jc*Sbar);
    _Z.setIdentity(_n,_n);
    updateNullspace(_Jc_Sbar_svd);
    //    sendMsg("Jc*Sbar*Z = "+toString((_Jc_Sbar*_Z).sum()));
    
#define DEBUG_POSE_TASKS
    // Solve the equality motion task
    // N=I
    // for i=1:K
    //      A = A_i*Sbar*N
    //      ddq_j = A^+ (b_i - A_i*ddq)
    //      ddq  += Sbar*ddq_j
    //      N    -= A^+ A
    // end
    bool jointLimitActive = _jointLimitTask!=NULL;
    if(jointLimitActive)
    {
        _jointLimitTask->update(robotState);
    }
    
    double tmp;
    list<MinJerkPDLinkPoseTask*>::iterator it;
    for(it=_equalityTasks.begin(); it!=_equalityTasks.end(); it++)
    {
        MinJerkPDLinkPoseTask& t = **it;
        t.update(robotState);
        
        t.getEqualityMatrix(_A_i);
        t.getEqualityVector(_a_i);
        _A = _A_i.rightCols(_n) - _A_i.leftCols<6>()*_Mb_inv_M_bj;
        _A *= _Z;
        _A_svd.compute(_A, _svdOptions);

        if(jointLimitActive)
        {
            // even if CI is constant, we need to update it every time because we overwrite it
            _jointLimitTask->getInequalityMatrix(_qp_motion1.CI);
            _jointLimitTask->getInequalityVector(_qp_motion1.ci0);
            
            _qp_motion1.H       = _A.transpose()*_A + _numericalDampingTask*MatrixXd::Identity(_n,_n);
            _qp_motion1.g       = -_A.transpose()*(_a_i - _A_i*_ddqDes);
            _qp_motion1.ci0     += _qp_motion1.CI*ddqDes_j;
            _qp_motion1.CI      *= _Z;
            for(int i=0; i<_qp_motion1.ci0.size(); i++)
                if(_qp_motion1.ci0(i)<-1e-10)
                    sendMsg(t.getName()+" ci0("+toString(i)+") = "+toString(_qp_motion1.ci0(i)));
            double res = solve_quadprog(_qp_motion1.H, _qp_motion1.g, _qp_motion1.CE.transpose(), _qp_motion1.ce0,
                                     _qp_motion1.CI.transpose(), _qp_motion1.ci0, _ddq_jDes,
                                     _qp_motion1.activeSet, _qp_motion1.activeSetSize);
            if(res == std::numeric_limits<double>::infinity())
            {
                cout<<t.getName()<<" QP could not be solved:\n"<<_qp_motion1.toString()<<endl;
                return false;
            }
            if(_qp_motion1.activeSetSize>0)
                sendMsg(t.getName()+". Active joint limits: "+toString(_qp_motion1.activeSet.head(_qp_motion1.activeSetSize).transpose()));
        }
        else
        {
            _ddq_jDes   = svdSolveWithDamping(_A_svd, _a_i - _A_i*_ddqDes, _numericalDampingTask);
        }
        if((tmp=(_ddq_jDes-_Z*_ddq_jDes).norm())>1e-10)
            sendMsg(t.getName()+". Part of ddq_jDes outside the null space of previous tasks: "+toString(tmp));
        ddqDes_b    -= _Mb_inv_M_bj*_ddq_jDes;
        ddqDes_j    += _ddq_jDes;
        updateNullspace(_A_svd);
        
#ifdef  DEBUG_POSE_TASKS
        _jointLimitTask->setDdqDes(ddqDes_j);
//        sendMsg(t.getName()+" Jacobian =\n"+toString(_A_i));
//        int nzsv = _A_svd.nonzeroSingularValues();
//        sendMsg(t.getName()+". "+toString(nzsv)+"-th sing val = "+toString(_A_svd.singularValues()(nzsv-1)));
//        sendMsg(t.getName()+". Contact constr error = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
//        sendMsg(t.getName()+" ||A*x-b|| =    "+toString((_A_i*_ddqDes-_a_i).norm()));
#endif
    }

//    sendMsg("Left arm ddq pre posture task:  "+toString(ddqDes_j.segment(3,5).transpose()));
    START_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    {
        if(jointLimitActive)
        {
            _jointLimitTask->getInequalityMatrix(_qp_posture.CI);
            _jointLimitTask->getInequalityVector(_qp_posture.ci0);
            
            _qp_posture.H       = _Z + _numericalDampingTask*MatrixXd::Identity(_n,_n);
            _qp_posture.g       = -_Z*(_ddq_jPosture - ddqDes_j);
            _qp_posture.ci0     += _qp_posture.CI*ddqDes_j;
            _qp_posture.CI      *= _Z;
            for(int i=0; i<_qp_posture.ci0.size(); i++)
                if(_qp_posture.ci0(i) < -1e-10)
                    cout<<"Posture ci0("+toString(i)+") = "+toString(_qp_posture.ci0(i))<<endl;
            double res = solve_quadprog(_qp_posture.H, _qp_posture.g, _qp_posture.CE.transpose(), _qp_posture.ce0,
                                        _qp_posture.CI.transpose(), _qp_posture.ci0, _ddq_jDes,
                                        _qp_posture.activeSet, _qp_posture.activeSetSize);
            if(res == std::numeric_limits<double>::infinity())
            {
                cout<<"Posture QP could not be solved:\n"<<_qp_posture.toString()<<endl;
//                _ddq_jDes.setZero();
                return false;
            }
            if(_qp_posture.activeSetSize>0)
                sendMsg("Posture task. Active joint limits: "+toString(_qp_posture.activeSet.head(_qp_posture.activeSetSize).transpose()));
        }
        else
        {
            if(_useNullspaceBase)
                _ddq_jDes   = _Z * (_Z.transpose() * _ddq_jPosture);
            else
                _ddq_jDes   = _Z * _ddq_jPosture;
        }
        
        if((tmp=(_ddq_jDes-_Z*_ddq_jDes).norm())>1e-10)
            sendMsg("Posture task. Part of ddq_jDes outside the null space of previous tasks: "+toString(tmp));
        ddqDes_b    -= _Mb_inv_M_bj*_ddq_jDes;
        ddqDes_j    += _ddq_jDes;
    }
    STOP_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    
    torques     = M_a*_ddqDes + h_j - Jc_j.transpose()*_fcDes;
    
//    sendMsg("fcDes              = "+toString(_fcDes,1));
//    sendMsg("Left arm ddq post posture task: "+toString(ddqDes_j.segment(3,5).transpose()));
//    sendMsg("Left arm ddq des posture task:      "+toString(_ddq_jPosture.segment(3,5).transpose()));
    //    sendMsg("ddq_jDes     = "+toString(_ddq_jDes,1));
    //    sendMsg("ddq_jPosture        = "+jointToString(_ddq_jPosture,1));
    sendMsg("Base dynamics error  = "+toString((M_u*_ddqDes+h_b-Jc_b.transpose()*_fcDes).norm()));
//    sendMsg("Joint dynamics error = "+toString((M_a*_ddqDes+h_j-Jc_j.transpose()*_fcDes-torques).norm()));
    sendMsg("Contact constr error = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
    

#ifdef  DEBUG_POSE_TASKS
    for(it=_equalityTasks.begin(); it!=_equalityTasks.end(); it++)
    {
        MinJerkPDLinkPoseTask& t = **it;
        t.getEqualityMatrix(_A_i);
        t.getEqualityVector(_a_i);
        sendMsg(t.getName()+" ||A*x-b|| =    "+toString((_A_i*_ddqDes-_a_i).norm()));
    }
#endif
    
    return true;
}

//**************************************************************************************************
bool wbiStackOfTasks::computeNullspaceProj(RobotState& robotState, Eigen::VectorRef torques)
{
    sendMsg("computeNullspaceProj");
    
    // compute ddq that is consistent with contact constraints
    SVD _Jc_svd;
    _Jc_svd.compute(_Jc, ComputeThinU|ComputeThinV);
    _ddqDes             = - svdSolveWithDamping(_Jc_svd, _dJcdq, _numericalDampingConstr);
    
    sendMsg("-Jcpinv*dJc*dq = "+toString(_ddqDes,1));
    
    // compute postural task
    int r = (_Jc_svd.singularValues().array()>PINV_TOL).count();
    MatrixRXd Nc        = MatrixRXd::Identity(_n+6,_n+6);
    Nc                  -= _Jc_svd.matrixV().leftCols(r) * _Jc_svd.matrixV().leftCols(r).transpose();
    MatrixRXd NcSTpinvD = pinvDampedEigen(Nc.rightCols(_n), _numericalDampingDyn);
    _ddqDes             += NcSTpinvD.transpose()*(_ddq_jPosture - _ddqDes.tail(_n));
    
    // compute torques
    torques             = NcSTpinvD * (_M*_ddqDes + _h);

    
//
//    sendMsg("ddqDes1 + NcSTpinvD^T*(_ddq_jPosture - ddqDes1) = "+toString(ddqDes,1));
//    sendMsg("(_M*ddqDes + _h)= "+toString((_M*ddqDes + _h),1));
    VectorXd torques_np6 = VectorXd::Zero(_n+6);
    torques_np6.tail(_n) = torques;
    _fcDes = svdSolveTransposeWithDamping(_Jc_svd, _M*_ddqDes+_h-torques_np6, _numericalDampingDyn);
    sendMsg("Dynamics error           = "+toString((Nc*(_M*_ddqDes+_h-torques_np6)).norm()));
    sendMsg("Contact constr error     = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
    sendMsg("Rank of Jc:                "+toString(r));
    
    return true;
}

//**************************************************************************************************
bool wbiStackOfTasks::computeComPosture(RobotState& robotState, Eigen::VectorRef torques)
{
    sendMsg("computeComPosture");
    
    START_PROFILING(PROFILE_FORCE_QP_MOMENTUM);
    {
        _qp_force.A     = _X.topRows<3>();
        _qp_force.a     = _momentumDes.head<3>();
        _qp_force.w     = _numericalDampingTask*_fWeights;
        _qp_force.CI    = _B;
        _qp_force.ci0   = _b;
//        sendMsg("H diag: "+toString(_qp_force.H.diagonal(),1));
//        sendMsg("f weights: "+toString(1e3*_fWeights,1));
//        sendMsg("X*N_X = "+toString((_X*_N_X).maxCoeff())+" "+toString((_X*_N_X).minCoeff()));
    }
    STOP_PROFILING(PROFILE_FORCE_QP_MOMENTUM);

    //    sendMsg("momentumDes "+toString(_momentumDes,1));
    //    cout<< "QP gradient "<<toString(_qp_force.g,1)<<endl;
    double res;
    START_PROFILING(PROFILE_FORCE_QP);
    {
        res     = _qp_force.solveQP();
        _fcDes  = _qp_force.x;
        if(res == std::numeric_limits<double>::infinity())
            return false;
        if(_qp_force.activeSetSize>0)
            sendMsg("Active constraints: "+toString(_qp_force.activeSet.head(_qp_force.activeSetSize).transpose()));
    }
    STOP_PROFILING(PROFILE_FORCE_QP);
    sendMsg("Linear momentum error  = "+toString((_X.topRows<3>()*_fcDes - _momentumDes.head<3>()).norm()));
    
    //*********************************
    // COMPUTE DESIRED ACCELERATIONS
    //*********************************
    
    // Compute ddq and f0 (i.e. y) that respect dynamics and the contact constraints
    MatrixRXd D = MatrixRXd::Zero(6+_k, _n+6+_k);
    MatrixRXd Dpinv(_n+6+_k, _k+6), DpinvD(_n+6+_k, _k+6);
    VectorXd d(6+_k);
    
    _X_svd.compute(_X.topRows<3>(), ComputeThinU|ComputeThinV);
    int r = (_X_svd.singularValues().array()>PINV_TOL).count();
    _N_X.setIdentity();
    _N_X -= _X_svd.matrixV().leftCols(r)*_X_svd.matrixV().leftCols(r).transpose();
    
    D.topLeftCorner(6, _n+6)        = M_u;
    D.bottomLeftCorner(_k, _n+6)    = _Jc;
    D.topRightCorner(6, _k)         = -Jc_b.transpose()*_N_X;
    d.head<6>()                     = Jc_b.transpose()*_fcDes - h_b;
    d.tail(_k)                      = -_dJcdq;
    pinvDampTrunc(D, PINV_TOL, _numericalDampingDyn, Dpinv, DpinvD);
    VectorXd y                      = DpinvD*d;
    
    START_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    {
        MatrixRXd N_D           = MatrixRXd::Identity(_n+6+_k,_n+6+_k) - Dpinv*D;
        MatrixRXd Jp_NDpinvD    = pinvDampedEigen(N_D.middleRows(6,_n), _numericalDampingTask);
        y                       += Jp_NDpinvD*(_ddq_jPosture - y.segment(6, _n));
    }
    STOP_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    
    _ddqDes     = y.head(_n+6);
    _fcDes     += _N_X*y.tail(_k);
    torques     = M_a*_ddqDes + h_j - Jc_j.transpose()*_fcDes;
    
    sendMsg("fcDes              = "+toString(_fcDes,1));
    //    sendMsg("ddq_jDes     = "+toString(_ddq_jDes,1));
    //    sendMsg("ddq_jPosture        = "+jointToString(_ddq_jPosture,1));
    sendMsg("Base dynamics error  = "+toString((M_u*_ddqDes+h_b-Jc_b.transpose()*_fcDes).norm()));
    sendMsg("Joint dynamics error = "+toString((M_a*_ddqDes+h_j-Jc_j.transpose()*_fcDes-torques).norm()));
    sendMsg("Contact constr error = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
    
    return true;
}

//**************************************************************************************************
bool wbiStackOfTasks::computeMomentumPosture(RobotState& robotState, Eigen::VectorRef torques)
{
    sendMsg("computeMomentumPosture");
    
    // compute desired contact forces
    _fcDes = pinvDampedEigen(_X, _numericalDampingTask) * _momentumDes;
    
    // compute ddq that are consistent with base dynamics and contact constraints
    MatrixRXd D = MatrixRXd::Zero(6+_k,_n+6);
    MatrixRXd Dpinv(_n+6,_k+6), DpinvD(_n+6,_k+6);
    VectorXd d(6+_k);
    D.topRows<6>()      = M_u;
    D.bottomRows(_k)    = _Jc;
    d.head<6>()         = Jc_b.transpose()*_fcDes - h_b;
    d.tail(_k)          = -_dJcdq;
    pinvDampTrunc(D, PINV_TOL, _numericalDampingConstr, Dpinv, DpinvD);
    _ddqDes             = DpinvD*d;
    
    // postural task
    MatrixRXd N_D       = MatrixRXd::Identity(_n+6,_n+6) - Dpinv*D;
    MatrixRXd SN_DpinvD = pinvDampedEigen(N_D.bottomRows(_n), _numericalDampingConstr);
    _ddqDes             += SN_DpinvD*(_ddq_jPosture - _ddqDes.tail(_n));
    torques             = M_a*_ddqDes + h_j - Jc_j.transpose()*_fcDes;
    
//    sendMsg("ddqDes             = "+jointToString(_ddqDes,1));
//    sendMsg("DEBUG ddqDes       = "+jointToString(ddqDes,1));
//    sendMsg("DEBUG ddqDes1      = "+jointToString(ddqDes1,1));
//    sendMsg("DEBUG ddqDesErr    = "+toString((ddqDes-_ddqDes).norm()));
//    sendMsg("fcDes              = "+toString(_fcDes,1));
//    sendMsg("DEBUG fcDes        = "+toString(fcDes2,1));
//    sendMsg("DEBUG fcDesErr     = "+toString((fcDes2-_fcDes).norm()));
    
    sendMsg("Base dynamics error  = "+toString((M_u*_ddqDes+h_b-Jc_b.transpose()*_fcDes).norm()));
    sendMsg("Joint dynamics error = "+toString((M_a*_ddqDes+h_j-Jc_j.transpose()*_fcDes-torques).norm()));
    sendMsg("Contact constr error = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
    sendMsg("tau (mom+post)       = "+jointToString(torques,1));

    return true;
}


//**************************************************************************************************
bool wbiStackOfTasks::solveForceQP(RobotState& robotState)
{
    double res;
    START_PROFILING(PROFILE_FORCE_QP);
    {
        res         = _qp_force.solveQP(_numericalDampingConstr);
        ddqDes_j    = _qp_force.x.head(_n);
        _fcDes      = _qp_force.x.tail(_k);
        
        bool qpFailed = false;
        if(res == std::numeric_limits<double>::infinity())
        {
            cout<<"Force QP could not be solved:\n"<<_qp_force.toString()<<endl;
            qpFailed = true;
        }
        else
        {
            if(_qp_force.activeSetSize>0)
                sendMsg("Active constraints: "+toString(_qp_force.activeSet.head(_qp_force.activeSetSize).transpose()));
            VectorXd tmp = _qp_force.CI*_qp_force.x;
            for(int i=0; i<tmp.size(); i++)
                if(tmp(i)+_qp_force.ci0(i)<-ZERO_NUM)
                {
                    cout<<"Force QP inequality constraint "+toString(i)+" is violated by the solution\n";
                    cout<<tmp(i)<<" < "<< -_qp_force.ci0(i)<<endl;
                    qpFailed = true;
                }
            if(qpFailed)
            {
                cout<<"x = "<<_qp_force.x.transpose().format(matlabPrintFormat)<<endl;
                cout<<_qp_force.toString()<<endl;
            }
        }
        
        if(qpFailed)
        {
            cout<<"Trying to solve force QP assuming zero velocities.\n";
            robotState.dq.setZero();
            robotState.dqJ.setZero();
            robotState.vBase.setZero();
            _robot->computeGeneralizedBiasForces(robotState.qJ.data(), robotState.xBase,
                                                 robotState.dqJ.data(), robotState.vBase.data(),
                                                 robotState.g.data(), _h.data());
            _dJcdq.setZero();
            _jointLimitTask->update(robotState);
            _qp_force.ce0               = _dJcdq - Jc_b*(_Mb_inv*h_b);
            int n_in = _jointLimitTask->getNumberOfInequalities();
            _jointLimitTask->getInequalityVector(_qp_force.ci0.head(n_in));
            for(int i=0; i<_n;i++)
            {   // enlarge ddq limits so that ddq=0 doesn't violate any inequalities
                if(-_qp_force.ci0(2*i)>0.0)         // ddqMin > 0 ?
                {
                    cout<<"Saturate to 0 ddqMin "<<i<<", which was "<<-_qp_force.ci0(2*i)<<endl;
                    _qp_force.ci0(2*i)=0.0;
                }
                else if(_qp_force.ci0(2*i+1)<0.0)   // ddqMax < 0 ?
                {
                    cout<<"Saturate to 0 ddqMax "<<i<<", which was "<<_qp_force.ci0(2*i+1)<<endl;
                    _qp_force.ci0(2*i+1)=0.0;
                }
            }
            res         = _qp_force.solveQP(_numericalDampingConstr);
            if(res == std::numeric_limits<double>::infinity())
            {
                cout<<"Force QP with zero velocities COULD NOT be solved:\n"<<_qp_force.toString()<<endl;
            }
            else
            {
                cout<<"Force QP with zero velocities COULD be solved:\n"<<_qp_force.toString()<<endl;
                VectorXd tmp = _qp_force.CI*_qp_force.x;
                for(int i=0; i<tmp.size(); i++)
                    if(tmp(i)+_qp_force.ci0(i)<-ZERO_NUM)
                    {
                        cout<<"Force QP inequality constraint "+toString(i)+" is violated by the solution\n";
                        cout<<tmp(i)<<" < "<< -_qp_force.ci0(i)<<endl;
                        cout<<"x = "<<_qp_force.x.transpose().format(matlabPrintFormat)<<endl;
                    }
            }
            return false;
        }
    }
    STOP_PROFILING(PROFILE_FORCE_QP);
    
    // Compute base accelerations
    //     dv_b = Mb^{-1}*(Jc_b^T*f - M_bj*ddq_j - h_b);
    ddqDes_b    = _Mb_inv*(Jc_b.transpose()*_fcDes - M_bj*ddqDes_j - h_b);
    return true;
}

//**************************************************************************************************
void wbiStackOfTasks::updateNullspace(JacobiSVD<MatrixRXd>& svd)
{
    int r = (svd.singularValues().array()>PINV_TOL).count();
    if(_useNullspaceBase)
        _Z *= svd.matrixV().rightCols(svd.cols()-r);
    else
        _Z -= svd.matrixV().leftCols(r)*svd.matrixV().leftCols(r).transpose();
}

//**************************************************************************************************
void wbiStackOfTasks::computeMb_inverse()
{
    // @todo Improve this computation exploting structure of Mb
    _Mb_llt.compute(M_b);
    _Mb_inv.setIdentity();
    _Mb_llt.solveInPlace(_Mb_inv);
}

//**************************************************************************************************
void wbiStackOfTasks::init(RobotState& robotState)
{
    _postureTask->init(robotState);
    _momentumTask->init(robotState);
    for(list<ContactConstraint*>::iterator it=_constraints.begin(); it!=_constraints.end(); it++)
        (*it)->init(robotState);
    for(list<MinJerkPDLinkPoseTask*>::iterator it=_equalityTasks.begin(); it!=_equalityTasks.end(); it++)
        (*it)->init(robotState);
    resizeVariables();
}

//**************************************************************************************************
void wbiStackOfTasks::addConstraint(ContactConstraint& constraint)
{
    _constraints.push_back(&constraint);
    resizeVariables();
}

//**************************************************************************************************
void wbiStackOfTasks::resizeVariables()
{
    int n_in=0; // number of inequalities
    int n_eq;   // number of equalities
    int nVar;   // number of variables
    int m;      // size of the force QP task (3 if control CoM, 6 if control 6d momentum)
    
    // count constraints
    _k = 0;
    for(list<ContactConstraint*>::iterator it=_constraints.begin(); it!=_constraints.end(); it++)
    {
        _k   += (*it)->getSize();
        n_in += (*it)->getNumberOfInequalities();
    }
    
    // resize matrices and vectors
    _X.setZero(6,_k);
    _X_svd = SVD(6, _k, ComputeThinU|ComputeThinV);
    _N_X.setZero(_k,_k);
    _B.setZero(n_in,_k);
    _b.setZero(n_in);
    _fWeights.setZero(_k);
    _Jc.setZero(_k,_n+6);
    _dJcdq.setZero(_k);
    _fcDes.setZero(_k);
    
    // resize force-QP variables
    if(_ctrlAlg==WBR_CTRL_ALG_MOMENTUM_SOT_SAFE)
    {
        nVar = _k+_n;
        if(_jointLimitTask!=NULL)
            n_in += _jointLimitTask->getNumberOfInequalities();
        n_eq = _k;
        m = 6;
    }
    else if(_ctrlAlg==WBR_CTRL_ALG_COM_SOT)
    {
        nVar = _k+_n;
        if(_jointLimitTask!=NULL)
            n_in += _jointLimitTask->getNumberOfInequalities();
        n_eq = _k;
        m = 3;
    }
    else
    {
        nVar = _k;
        n_eq = 0;
        m = _ctrlAlg==WBR_CTRL_ALG_COM_POSTURE ? 3 : 6;
    }
    _qp_force.resize(nVar, n_eq, n_in, m);
    cout<<"Resize force QP with "<<nVar<<" variables, "<<n_eq<<" equalities, "<<n_in<<" inequalities, "<<m<<" task size\n";
    
    if(_ctrlAlg==WBR_CTRL_ALG_COM_SOT)
    {
        n_eq = 0;
        // resize first motion-task-QP variables
        _qp_motion1.resize(nVar, n_eq, n_in, 6);
        cout<<"Resize motion QP with "<<nVar<<" variables, "<<n_eq<<" equalities, "<<n_in<<" inequalities, "<<6<<" task size\n";
        // resize second motion-task-QP variables
        _qp_motion2.resize(nVar, n_eq, n_in, 6);
        // resize posture-QP variables
        _qp_posture.resize(nVar, n_eq, n_in, _n);
        cout<<"Resize posture QP with "<<nVar<<" variables, "<<n_eq<<" equalities, "<<n_in<<" inequalities, "<<_n<<" task size\n";
        _Z.setIdentity(nVar, nVar);
    }
    else
    {
        // resize first motion-task-QP variables
        n_in = _jointLimitTask==NULL ? 0 : _jointLimitTask->getNumberOfInequalities();
        n_eq = _k;
        _qp_motion1.resize(_n, n_eq, n_in, 6);
        
        // resize second motion-task-QP variables
        _qp_motion2.resize(_n, 0, n_in, 6);
        
        // resize posture-QP variables
        n_eq = _equalityTasks.empty() ? _k : 0;
        _qp_posture.resize(_n, n_eq, n_in, _n);
    }
}

//**************************************************************************************************
void wbiStackOfTasks::linkParameterToVariable(ParamTypeId paramType, ParamHelperServer* paramHelper, int paramId)
{
    switch(paramType)
    {
        case USE_NULLSPACE_BASE:
            _useNullspaceBase_paramId = paramId;
            paramHelper->linkParam(paramId, &_useNullspaceBase);
            paramHelper->registerParamValueChangedCallback(paramId, this);
            break;
        case DYN_NUM_DAMP:
            paramHelper->linkParam(paramId, &_numericalDampingDyn);
            break;
        case CONSTR_NUM_DAMP:
            paramHelper->linkParam(paramId, &_numericalDampingConstr);
            break;
        case TASK_NUM_DAMP:
            paramHelper->linkParam(paramId, &_numericalDampingTask);
            break;
        case CTRL_ALG:
            _ctrlAlg_paramId = paramId;
            paramHelper->linkParam(paramId, &_ctrlAlg);
            paramHelper->registerParamValueChangedCallback(paramId, this);
            break;
        default:
            cout<< "[wbiStackOfTasks::linkParameterToVariable] Trying to link a parameter that is not managed\n";
    }
}

//**************************************************************************************************
void wbiStackOfTasks::parameterUpdated(const ParamProxyInterface* pp)
{
    if(pp->id==_useNullspaceBase_paramId)
    {
        useNullspaceBase(_useNullspaceBase==1);
        return;
    }
    else if(pp->id==_ctrlAlg_paramId)
    {
        resizeVariables();
        return;
    }
    cout<< "[wbiStackOfTasks::parameterUpdated] Callback for a parameter that is not managed"
        << pp->name<< endl;
}

#ifdef DEBUG_FORWARD_DYNAMICS
//**************************************************************************************************
void wbiStackOfTasks::constrainedForwardDynamics(Eigen::Vector3d& g, Eigen::VectorConst torques,
                                                 wbi::Frame &xBase, Eigen::VectorRef qj,
                                                 Eigen::VectorRef dq, Eigen::VectorRef ddq)
{
    // compute constraint matrix and vector: A*ddq = b
    //    for(int i=0; i<_constrainedLinkIds.size(); i++)
    //    {
    //        _robot->computeJacobian(qj.data(), xBase, _constrainedLinkIds[i], _A.row(i*6).data());
    //        _robot->computeDJdq(qj.data(), xBase, dq.tail(_n).data(), dq.data(), _constrainedLinkIds[i], &_b[i*6]);
    //    }
    //    _b *= -1.0;
    
    // compute mass matrix and generalized bias forces
    //    _robot->computeMassMatrix(qj.data(), xBase, _M.data());
    //    _robot->computeGeneralizedBiasForces(qj.data(), xBase, dq.tail(_n).data(), dq.data(), g.data(), _h.data());
    
    // compute constraint solution: ddqBar = - Jc^+ * dJc * dq
    _tau_np6.setZero(_n+6);
    
    _Jc_svd.compute(_Jc, ComputeFullU | ComputeFullV);
    _ddqBar = -1.0*svdSolveWithDamping(_Jc_svd, _dJcdq, _numericalDampingConstr);
    
    // compute base of null space of constraint Jacobian
    int r = (_Jc_svd.singularValues().array()>PINV_TOL).count();
    _Zc = _Jc_svd.matrixV().rightCols(_n+6-r);
    
    // compute constrained accelerations ddq_c = (Z^T*M*Z)^{-1}*Z^T*(S^T*tau - h - M*ddqBar)
    _ZMZ = _Zc.transpose()*_M*_Zc;
    _ZMZ_chol.compute(_ZMZ);
    _tau_np6.tail(_n) = torques;
    _ddq_c = _Zc.transpose()*(_tau_np6 - _h - _M*_ddqBar);
    _ZMZ_chol.solveInPlace(_ddq_c);
    // compute joint accelerations
    ddq = _ddqBar + _Zc*_ddq_c;
    
    //    getLogger().sendMsg("A*ddq-b = "+toString((_A*ddq-_b).norm()), MSG_STREAM_INFO);
    //    getLogger().sendMsg("ddqBar = "+jointToString(ddqBar,2), MSG_STREAM_INFO);
    //    getLogger().sendMsg("ddq_c  = "+toString(_ddq_c,2), MSG_STREAM_INFO);
    //    getLogger().sendMsg("ddq_c RHS = "+toString(_Zc.transpose()*(_tau_np6 - _h - _M*ddqBar),2), MSG_STREAM_INFO);
    //    getLogger().sendMsg("Nc*(M*ddq+h-S^T*tau) = "+toString((_Zc.transpose()*(_M*ddq+_h-_tau_np6)).norm()), MSG_STREAM_INFO);
    //    getLogger().sendMsg("Jc^T*f = "+toString(_M*ddq+_h-_tau_np6, 1), MSG_STREAM_INFO);
    //    getLogger().sendMsg("ddq integration: "+jointToString(WBR_RAD2DEG*ddq,1), MSG_STREAM_INFO);
    //    getLogger().sendMsg("Jc:\n"+toString(_Jc,1), MSG_STREAM_INFO);
}
#endif

