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
        USE_NULLSPACE_BASE,
        NUMERICAL_DAMPING
    };
    
protected:
    double          _numericalDamping;          /// damping factor to use in solver
    int             _numericalDamping_paramId;  /// id of the parameter associated to _numericalDamping
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
    Eigen::JacobiSVD<Eigen::MatrixRXd>  _A_svd;
    
    Eigen::MatrixRXd                _Jc_Sbar;   /// Jc projected in nullspace of base dynamics
    Eigen::JacobiSVD<Eigen::MatrixRXd>  _Jc_Sbar_svd;   /// svd of Jc*Sbar
    
    Eigen::MatrixRXd            _X;             /// matrix mapping constraint forces into momentum derivative
    Eigen::MatrixRXd            _Jc;            /// constraint Jacobian
    Eigen::VectorXd             _dJcdq;         /// dJc*dq
    Eigen::VectorXd             _fcDes;         /// desired constraint forces (result of QP)
    Eigen::Vector6d             _momentumDes;   /// desired momentum
    Eigen::VectorXd             _ddq_jPosture;  /// desired acceleration given by posture task
    Eigen::VectorXd             _ddqDes;        /// desired accelerations (n+6)
    
    
    //    minimize      0.5 * x H x + g x
    //    subject to    CE^T x + ce0 = 0
    //                  CI^T x + ci0 >= 0
    struct
    {
        Eigen::MatrixXd H;      /// Hessian
        Eigen::VectorXd g;      /// gradient
        Eigen::MatrixXd CE;     /// equality constraint matrix
        Eigen::VectorXd ce0;    /// equality constraint vecotr
        Eigen::MatrixRXd CI;     /// inequality constraint matrix
        Eigen::VectorXd ci0;    /// inequality constraint vector
        Eigen::VectorXi activeSet;  /// vector containing the indexes of the active inequalities
        int activeSetSize;
    } _qpData;
    
    /** Compute the inverse of the matrix Mb. */
    void computeMb_inverse();
    
    /** Update the null-space base/projector (contained in _Z) by projecting it
      * in the nullspace of the specified matrix. */
    void updateNullspace(Eigen::JacobiSVD<Eigen::MatrixRXd>& svd);
    
    void sendMsg(const std::string &s, MsgType type=MSG_STREAM_INFO)
    {
        getLogger().sendMsg("[wbiStackOfTasks] "+s, type);
    }
    
public:
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
    { _jointLimitTask=&taskJL; }
    
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
    
    void setNumericalDamping(double d){ _numericalDamping=d; }
    
    double getNumericalDamping(){ return _numericalDamping; }
};
    

Eigen::VectorXd svdSolveWithDamping(const Eigen::JacobiSVD<Eigen::MatrixRXd>& svd, Eigen::VectorConst b, double damping=0.0);



/** Compute the truncated pseudoinverse of the specified matrix A. 
  * This version of the function takes addtional input matrices to avoid allocating memory and so improve 
  * the efficiency of the computation. 
  * @param A Input mXn matrix.
  * @param tol Input threshold for the singular values of the truncated pseudoinverse.
  * @param Spinv Output kXk matrix (with k=min(m,n)), truncated pseudoinverse of the singular value matrix of A. 
  * @param Apinv Output nXm matrix, truncated pseudoinverse of A.
  * @param sv Output (optional) k-dim vector (with k=min(m,n)), singular values of A. */
void pinvTrunc(const Eigen::MatrixRXd &A, double tol, Eigen::MatrixRXd &Apinv, Eigen::MatrixRXd &Spinv, Eigen::VectorXd &sv);

/** Compute two different pseudoinverses of the specified matrix A: a truncated pseudoinverse and a
  * damped pseudoinverse. The difference between the two versions is that the truncated version sets to zero
  * all the singular values that are less than a certain threshold (tol), whereas the damped version
  * uses computes this expression: \f[ A^+ = A^T(AA^T+\lambda I)^{-1}\f], where \f[ \lambda \f] is the damping 
  * factor. Both pseudoinverses are computed from the singular value decomposition of A.
  * @param A Input mXn matrix.
  * @param tol Input threshold for the singular values of the truncated pseudoinverse.
  * @param damp Input damping factor for the damped pseudoinverse.
  * @param Apinv Output nXm matrix, truncated pseudoinverse of A.
  * @param ApinvDamp Output nXm matrix, damped pseudoinverse of A.
  * @param sv Output (optional) k-dim vector (with k=min(m,n)), singular values of A. */
void pinvDampTrunc(const Eigen::MatrixRXd &A, double tol, double damp, Eigen::MatrixRXd &Apinv, Eigen::MatrixRXd &ApinvDamp, Eigen::MatrixRXd &Spinv, Eigen::MatrixRXd &SpinvD, Eigen::VectorXd &sv);
    
Eigen::MatrixRXd nullSpaceProjector(const Eigen::Ref<Eigen::MatrixRXd> A, double tol);
    
Eigen::MatrixRXd pinvDampedEigen(const Eigen::Ref<Eigen::MatrixRXd> &A, double damp);

    
/** Tolerance for considering two values equal */
const double ZERO_TOL = 1e-5;
    
void assertEqual(const Eigen::MatrixRXd &A, const Eigen::MatrixRXd &B, std::string testName, double tol = ZERO_TOL);

void testFailed(std::string testName);

}

#endif
