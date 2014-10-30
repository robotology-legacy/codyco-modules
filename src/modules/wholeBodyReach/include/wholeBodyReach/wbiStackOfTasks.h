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

#include <wholeBodyReach/wholeBodyReachConstants.h>
#include <wholeBodyReach/wbiMinJerkTasks.h>
#include <wholeBodyReach/wbiConstraints.h>
#include <wholeBodyReach/Logger.h>


namespace wholeBodyReach
{
    
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
    
    Eigen::MatrixRXd                _Mb_inv;    /// inverse of the 6x6 base mass matrix
    Eigen::LLT<Eigen::MatrixRXd>    _Mb_llt;    /// Cholesky decomposition of Mb
    Eigen::MatrixRXd                _Mb_inv_M_bj;   /// _Mb_inv*M_bj
    
    Eigen::VectorXd                 _ddq_jDes;  /// desired joint accelerations
    Eigen::MatrixRXd                _Z;         /// null-space basis/projector
    
    /// HIERARCHY OF EQUALITY RESOLUTION 
    Eigen::MatrixRXd                _A;
    Eigen::MatrixRXd                _A_i;
    Eigen::VectorXd                 _b_i;
    Eigen::SVD                      _A_svd;
    
    Eigen::MatrixRXd                _Jc_Sbar;       /// Jc projected in nullspace of base dynamics
    Eigen::SVD                      _Jc_Sbar_svd;   /// svd of Jc*Sbar
    
    Eigen::MatrixRXd            _X;             /// matrix mapping constraint forces into momentum derivative
    Eigen::SVD                  _X_svd;         /// svd of X
    Eigen::MatrixRXd            _N_X;           /// nullspace projector of _X
    
    Eigen::VectorXd             _fWeights;      /// penalty weights for _fcDes
    Eigen::MatrixRXd            _W;             /// force weight matrix
    Eigen::MatrixRXd            _Jc;            /// constraint Jacobian
    Eigen::VectorXd             _dJcdq;         /// dJc*dq
    Eigen::VectorXd             _fcDes;         /// desired constraint forces (result of QP)
    Eigen::Vector6d             _momentumDes;   /// desired momentum
    Eigen::VectorXd             _ddq_jPosture;  /// desired acceleration given by posture task
    Eigen::VectorXd             _ddqDes;        /// desired accelerations (n+6)
    
    
    //    minimize      0.5 * x H x + g x
    //    subject to    CE^T x + ce0 = 0
    //                  CI^T x + ci0 >= 0
    struct QpData
    {
        Eigen::MatrixXd H;      /// Hessian
        Eigen::VectorXd g;      /// gradient
        Eigen::MatrixXd CE;     /// equality constraint matrix
        Eigen::VectorXd ce0;    /// equality constraint vecotr
        Eigen::MatrixRXd CI;     /// inequality constraint matrix
        Eigen::VectorXd ci0;    /// inequality constraint vector
        Eigen::VectorXi activeSet;  /// vector containing the indexes of the active inequalities
        int activeSetSize;
    };
    
    QpData _qpData_f;       /// data for the force QP
    QpData _qpData_ddq;     /// data for the joint acceleration QP
    
    /** Compute the inverse of the matrix Mb. */
    void computeMb_inverse();
    
    /** Update the null-space base/projector (contained in _Z) by projecting it
      * in the nullspace of the specified matrix. */
    void updateNullspace(Eigen::JacobiSVD<Eigen::MatrixRXd>& svd);
    
    void sendMsg(const std::string &s, MsgType type=MSG_STREAM_INFO)
    {
        getLogger().sendMsg("[wbiStackOfTasks] "+s, type);
    }
    
//    WBR_CTRL_ALG_MOMENTUM_SOT       = 0,
//    WBR_CTRL_ALG_NULLSPACE_PROJ     = 1,
//    WBR_CTRL_ALG_COM_POSTURE        = 2,
//    WBR_CTRL_ALG_MOMENTUM_POSTURE   = 3,
    virtual bool computeMomentumSoT(RobotState& robotState, Eigen::VectorRef torques);
    virtual bool computeNullspaceProj(RobotState& robotState, Eigen::VectorRef torques);
    virtual bool computeComPosture(RobotState& robotState, Eigen::VectorRef torques);
    virtual bool computeMomentumPosture(RobotState& robotState, Eigen::VectorRef torques);
    
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
    { _equalityTasks.push_back(&task); }
    
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
        _qpData_ddq.CI  = taskJL.getInequalityMatrix();
        _qpData_ddq.ci0 = taskJL.getInequalityVector();
        _qpData_ddq.activeSet = Eigen::VectorXi::Constant(_qpData_ddq.ci0.size(), -1);
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
