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

#ifndef WHOLE_BODY_REACH_WBI_SOT
#define WHOLE_BODY_REACH_WBI_SOT

#include <Eigen/Core>               // import most common Eigen types
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>           // Timer
#include <paramHelp/paramHelperServer.h>
#include <vector>
#include <list>
#include <string>
#include <iostream>
#include <iterator>

#include <wholeBodyReach/eiquadprog2011.hpp>

#include <wholeBodyReach/wholeBodyReachConstants.h>
#include <wholeBodyReach/wbiMinJerkTasks.h>
#include <wholeBodyReach/wbiConstraints.h>
#include <wholeBodyReach/Logger.h>


namespace wholeBodyReach
{
    
//    minimize      0.5* || A x - a ||^2 + diag(w) ||x||^2
//    subject to    CE x + ce0 = 0
//                  CI x + ci0 >= 0
class QpData
{
private:
    Eigen::SVD       CE_svd;    /// svd of CE
    int              svdOpt;    /// specify whether to compute full/thin U/V
    Eigen::VectorXd  x_bar;     /// a solution of the equality constraints
    
    Eigen::MatrixXd CE_empty;   /// empty matrix
    Eigen::VectorXd ce0_empty;  /// empty vector
    
    Eigen::MatrixRXd CI_p;      /// projected inequality constraint matrix
    Eigen::VectorXd ci0_p;      /// projected inequality constraint vector
    
    Eigen::MatrixRXd A_p;   /// projected matrix of least-square form (H = A^T*A + diag(w))
    Eigen::SVD       A_svd; /// svd of the matrix A
    Eigen::VectorXd  a_p;   /// projected vector of least-square form ||A*x-a|| (g = -A^T*a)
    
public:
    // @todo move H and g to private section
    Eigen::MatrixXd H;          /// Hessian
    Eigen::VectorXd g;          /// gradient
    Eigen::VectorXd x;          /// solution of the QP
    
    Eigen::MatrixRXd CE;        /// equality constraint matrix
    Eigen::MatrixRXd N_CE;      /// null-space projector of CE
    Eigen::VectorXd  ce0;       /// equality constraint vecotr
    
    
    Eigen::MatrixRXd CI;        /// inequality constraint matrix
    Eigen::VectorXd ci0;        /// inequality constraint vector
    Eigen::VectorXi activeSet;  /// vector containing the indexes of the active inequalities
    int activeSetSize;
    
    Eigen::MatrixRXd A;     /// matrix of least-square form (H = A^T*A + diag(w))
    Eigen::VectorXd  a;     /// vector of least-square form ||A*x-a|| (g = -A^T*a)
    Eigen::VectorXd  w;     /// diagonal elements of regularization matrix
    
    double solveQP(double numericalDamping=0.0)
    {
        double res;
        if(CE.rows()>0)
        {
            // solve equality constraints
            CE_svd.compute(CE, svdOpt);
            x_bar = - svdSolveWithDamping(CE_svd, ce0, numericalDamping);
            
            // compute null-space projector
            int r = (CE_svd.singularValues().array()>PINV_TOL).count();
            N_CE.setIdentity();
            N_CE  -= CE_svd.matrixV().leftCols(r) * CE_svd.matrixV().leftCols(r).transpose();
            
            // update least-square form
            // || A*x - a || => || A*(x_bar+N_CE*z) - a || = || A*N_CE*z - (a-A*x_bar) ||
            a_p = a - A*x_bar;
            A_p = A*N_CE;
            
            
            // update inequalities
            // CI*x + ci0 => CI*(x_bar+N_CE*z) + ci0 => CI*N_CE*z + (ci0 + CI*x_bar)
            if(CI.rows()>0)
            {
                ci0_p = ci0 + CI*x_bar;
                CI_p  = CI*N_CE;
            }
            
            // update Hessian and gradient
            H = A_p.transpose()*A_p;
            H.diagonal() += w;
            g = -1.0*(A_p.transpose()*a_p);
            
            res = solve_quadprog(H, g, CE_empty, ce0_empty, CI_p.transpose(), ci0_p,
                                 x, activeSet, activeSetSize);
            if(res!=std::numeric_limits<double>::infinity())
                x = x_bar + N_CE*x;
        }
        else
        {
            // update Hessian and gradient
            H = A.transpose()*A;
            H.diagonal() += w;
            g = -1.0*(A.transpose()*a);
            
            res = solve_quadprog(H, g, CE_empty, ce0_empty, CI.transpose(), ci0,
                                 x, activeSet, activeSetSize);
        }
        return res;
    }
    
    /** Compute the null-space projector for the whole QP, that takes into
     * account both the equality constraints and the cost function.
     * After calling this function the null-space projector is stored in N_CE.
     * This is supposed to be called after solveQP().
     */
    void computeQpNullSpace()
    {
        if(CE.rows()>0)
        {
            A_svd.compute(A_p, svdOpt);
        }
        else
        {
            A_svd.compute(A, svdOpt);
            N_CE.setIdentity();
        }
        int r = (A_svd.singularValues().array()>PINV_TOL).count();
        N_CE  -= A_svd.matrixV().leftCols(r) * A_svd.matrixV().leftCols(r).transpose();
    }
    
    /** Resize all the variables.
     * @param nVar Number of variables of the QP
     * @param nEq Number of equalities of the QP
     * @param nIn Number of inequalities of the QP
     * @param nA  Number of rows of the least-square form ||A*x-a||
     */
    void resize(int nVar, int nEq, int nIn, int nA)
    {
        x.setZero(nVar);
        H.setZero(nVar, nVar);
        g.setZero(nVar);
        
        CE.setZero(nEq, nVar);
        svdOpt = Eigen::ComputeThinU | Eigen::ComputeThinV;
        if(nVar>0 && nEq>0)
            CE_svd.compute(CE, svdOpt);
        N_CE.setIdentity(nVar,nVar);
        ce0.setZero(nEq);
        
        CI.setZero(nIn, nVar);
        ci0.setZero(nIn);
        CI_p.setZero(nIn, nVar);
        ci0_p.setZero(nIn);
        activeSet = Eigen::VectorXi::Constant(nIn, -1);
        activeSetSize = 0;
        
        A.setZero(nA, nVar);
        A_p.setZero(nA, nVar);
        if(nVar>0 && nA>0)
            A_svd.compute(A,svdOpt);
        a.setZero(nA);
        a_p.setZero(nA);
        w.setZero(nVar);
        
        CE_empty.resize(0,0);
        ce0_empty.resize(0);
    }
    
    std::string toString()
    {
        std::stringstream ss;
        ss<<"****************************************************************\n";
        //            ss<<"H=    "<<H.format(matlabPrintFormat)<<endl;
        //            ss<<"g=    "<<g.transpose().format(matlabPrintFormat)<<endl;
        ss<<"A=    "<<A.format(matlabPrintFormat)<<endl;
        ss<<"a=    "<<a.transpose().format(matlabPrintFormat)<<endl;
        ss<<"w=    "<<w.transpose().format(matlabPrintFormat)<<endl<<endl;
        ss<<"CE=   "<<CE.format(matlabPrintFormat)<<endl;
        ss<<"ce0=  "<<ce0.transpose().format(matlabPrintFormat)<<endl<<endl;
        ss<<"CI=   "<<CI.format(matlabPrintFormat)<<endl;
        ss<<"ci0=  "<<ci0.transpose().format(matlabPrintFormat)<<endl;
        ss<<"****************************************************************\n";
        return ss.str();
    }
};

    
/** A simplified ad-hoc version of the classical stack of task.
  * The following assumptions hold:
  * - the robot is floating-base and possibly constrained
  * - there is one (and only one) task for the control of the momentum
  * - there is one (and only one) task for the joint limits
  * - there is one (and only one) task for the posture
  * - there is an arbitrary number of generic equality motion tasks
 */
class wbiStackOfTasks : public paramHelp::ParamValueObserver
{
public:
    /// This enum defines all the parameters of this class
    enum ParamTypeId
    {
        USE_NULLSPACE_BASE, /// define whether to use null-space projectors or basis
        DYN_NUM_DAMP,       /// numerical damping used when solving dynamics
        CONSTR_NUM_DAMP,    /// numerical damping used when solving constraints
        TASK_NUM_DAMP,      /// numerical damping used when solving tasks
        CTRL_ALG            /// define which control algorithm to use
    };
    
public:
    WholeBodyReachCtrlAlgorithm     _ctrlAlg;   /// the id of the control algorithm to use
    int             _ctrlAlg_paramId;           /// id of the parameter associated to _ctrlAlg
    double          _numericalDampingDyn;       /// damping factor when solving dynamics
    double          _numericalDampingConstr;    /// damping factor when solving constraints
    double          _numericalDampingTask;      /// damping factor when solving tasks
    int             _useNullspaceBase;          /// 1: use base, 0: use projector
    int             _useNullspaceBase_paramId;  /// id of the parameter associated to _useNullspaceBase
    int             _svdOptions;                /// specify whether to compute full/thin U/V
    
    std::list<MinJerkPDLinkPoseTask*>   _equalityTasks;
    MinJerkPDMomentumTask*              _momentumTask;
    std::list<ContactConstraint*>       _constraints;
    JointLimitTask*                     _jointLimitTask;
    MinJerkPDPostureTask*               _postureTask;
    
    wbi::wholeBodyInterface*        _robot;
    int                             _n;         /// number of degrees of freedom of the robot
    int                             _k;         /// number of constraints
    
    Eigen::MatrixRXd                _M;         /// floating-base mass matrix (n+6)x(n+6)
    Eigen::VectorXd                 _h;         /// generalized bias forces (n+6)
    
    Eigen::MatrixR6d                _Mb_inv;    /// inverse of the 6x6 base mass matrix
    Eigen::LLT<Eigen::MatrixR6d>    _Mb_llt;    /// Cholesky decomposition of Mb
    Eigen::MatrixRXd                _Mb_inv_M_bj;   /// _Mb_inv*M_bj
    Eigen::MatrixRXd                _Mb_inv_J_cbT;  /// _Mb_inv*Jc_b^T
    
    Eigen::VectorXd                 _ddq_jDes;  /// desired joint accelerations
    Eigen::MatrixRXd                _Z;         /// null-space basis/projector
    
    /// HIERARCHY OF EQUALITY RESOLUTION 
    Eigen::MatrixRXd                _A;
    Eigen::MatrixRXd                _A_i;
    Eigen::VectorXd                 _a_i;
    Eigen::SVD                      _A_svd;
    
    Eigen::MatrixRXd                _Jc_Sbar;       /// Jc projected in nullspace of base dynamics
    Eigen::SVD                      _Jc_Sbar_svd;   /// svd of Jc*Sbar
    
    Eigen::MatrixRXd            _X;             /// matrix mapping constraint forces into momentum derivative
    Eigen::SVD                  _X_svd;         /// svd of X
    Eigen::MatrixRXd            _N_X;           /// nullspace projector of _X
    Eigen::MatrixRXd            _B;             /// friction-cone inequality matrix B*f+b>0
    Eigen::VectorXd             _b;             /// friction-cone inequality vector B*f+b>0
    
    Eigen::VectorXd             _fWeights;      /// penalty weights for _fcDes
    Eigen::MatrixRXd            _Jc;            /// constraint Jacobian
    Eigen::VectorXd             _dJcdq;         /// dJc*dq
    Eigen::VectorXd             _fcDes;         /// desired constraint forces (result of QP)
    Eigen::Vector6d             _momentumDes;   /// desired momentum
    Eigen::VectorXd             _ddq_jPosture;  /// desired acceleration given by posture task
    Eigen::VectorXd             _ddqDes;        /// desired accelerations (n+6)
    
    QpData _qp_force;       /// force QP
    QpData _qp_motion1;     /// first motion-task QP
    QpData _qp_motion2;     /// second motion-task QP
    QpData _qp_posture;     /// postural-task QP
    
    bool solveForceQP(RobotState& robotState);
    
    /** Compute the inverse of the matrix Mb. */
    void computeMb_inverse();
    
    /** Update the null-space base/projector (contained in _Z) by projecting it
      * in the nullspace of the specified matrix. */
    void updateNullspace(Eigen::JacobiSVD<Eigen::MatrixRXd>& svd);
    
    /** Resize all the internal variables base on the current number of constraints
     * and the current control algorithm. */
    virtual void resizeVariables();
    
    void sendMsg(const std::string &s, MsgType type=MSG_STREAM_DEBUG)
    {
        getLogger().sendMsg("[wbiStackOfTasks] "+s, type);
    }
    
//    WBR_CTRL_ALG_MOMENTUM_SOT       = 0,
//    WBR_CTRL_ALG_NULLSPACE_PROJ     = 1,
//    WBR_CTRL_ALG_COM_POSTURE        = 2,
//    WBR_CTRL_ALG_MOMENTUM_POSTURE   = 3,
//    WBR_CTRL_ALG_MOMENTUM_SOT_SAFE  = 4,
//    WBR_CTRL_ALG_COM_SOT            = 5,
    virtual bool computeMomentumSoT(RobotState& robotState, Eigen::VectorRef torques);
    virtual bool computeMomentumSoT_safe(RobotState& robotState, Eigen::VectorRef torques);
    virtual bool computeNullspaceProj(RobotState& robotState, Eigen::VectorRef torques);
    virtual bool computeComPosture(RobotState& robotState, Eigen::VectorRef torques);
    virtual bool computeMomentumPosture(RobotState& robotState, Eigen::VectorRef torques);
    virtual bool computeComSoT(RobotState& robotState, Eigen::VectorRef torques);
    
//#define DEBUG_FORWARD_DYNAMICS
#ifdef DEBUG_FORWARD_DYNAMICS
public:
    Eigen::SVD _Jc_svd;
    Eigen::MatrixRXd _ZMZ, _Zc;
    Eigen::VectorXd _ddq_c, _ddqBar, _ddqFD, _tau_np6;
    Eigen::VectorXd _qj, _dq;
    wbi::Frame _xB;
    Eigen::Cholesky _ZMZ_chol;
    
    void constrainedForwardDynamics(Eigen::Vector3d& g, Eigen::VectorConst torques, wbi::Frame &xBase,
                                    Eigen::VectorRef qj, Eigen::VectorRef dq,
                                    Eigen::VectorRef ddq);
#endif
    
public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /** Constructor
      * @param robot Class to compute the robot dynamics and kinematics
      * @param useNullspaceBase If true the solver uses the basis of the nullspace, 
                                otherwise it uses nullspace projectors
     */
    wbiStackOfTasks(wbi::wholeBodyInterface* robot, bool useNullspaceBase=false);
    
    /** Update all tasks/constraints and compute the control
      * torques to send to the motors.
      * @param robotState The current state of the robot
      * @param torques Output control torques.
      */
    virtual bool computeSolution(RobotState& robotState, Eigen::VectorRef torques);
    
    /** Initialize the solver and the trajectory generator of the tasks.
      * To be called once before starting to call computeSolution. */
    virtual void init(RobotState& robotState);
    
    virtual void linkParameterToVariable(ParamTypeId paramType, paramHelp::ParamHelperServer* paramHelper, int paramId);
    
    /** Method called every time a parameter (for which a callback is registered) is changed. */
    virtual void parameterUpdated(const paramHelp::ParamProxyInterface *pp);
    
    /** Push the specified equality task at the end of the stack,
      * so that it becomes the lowest-priority task.
      * @param task The task to push.
      */
    virtual void pushEqualityTask(MinJerkPDLinkPoseTask& task)
    {
        _equalityTasks.push_back(&task);
        resizeVariables();
    }
    
    /** Add the specified constraint to the list of constraints.
      * The order of the constraints does not matter.
      * @param constraint The constraint to add.
      */
    virtual void addConstraint(ContactConstraint& constraint);
    
    /** Set the task for the control of the momentum.
      * If a task is already set, it is replaced.
      * @param The momentum control task.
      * @note The momentum task always takes the highest priority.
      */
    virtual void setMomentumTask(MinJerkPDMomentumTask& taskMomentum)
    { _momentumTask=&taskMomentum; }
    
    /** Set the joint-limit task. If a task is already set,
      * it is replaced.
      * @param taskJL The joint-limit task. 
      * @note The joint-limit task is actually an inequality constraint and
      *       it takes the highest priority in the stack.
      */
    virtual void setJointLimitTask(JointLimitTask& taskJL)
    {
        _jointLimitTask = &taskJL;
        resizeVariables();
    }
    
    /** Set the posture task, which always takes the lowest priority
      * in the stack. If a task is already set, it is replaced.
      * @param taskPosture The posture control task
      */
    virtual void setPostureTask(MinJerkPDPostureTask& taskPosture)
    { _postureTask=&taskPosture; }
    
    virtual void useNullspaceBase(bool b)
    {
        _useNullspaceBase = b? 1 : 0;
        if(b)
            _svdOptions = Eigen::ComputeFullU | Eigen::ComputeFullV;
        else
            _svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;
    }
};
    
}

#endif
