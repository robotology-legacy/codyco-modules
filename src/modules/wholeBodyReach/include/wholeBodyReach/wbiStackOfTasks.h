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
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>           // Timer
#include <vector>
#include <list>
#include <string>
#include <iostream>
#include <iterator>

#include <wholeBodyReach/wholeBodyReachConstants.h>
#include <wholeBodyReach/wbiMinJerkTasks.h>


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
class wbiStackOfTasks
{
protected:
    std::list<MinJerkPDLinkPoseTask*>   _equalityTasks;
    MinJerkPDMomentumTask*              _momentumTask;
    std::list<ContactConstraint*>       _constraints;
    JointLimitTask*                     _jointLimitTask;
    MinJerkPDPostureTask*               _postureTask;
    
    wbi::wholeBodyInterface*        _robot;
    int                             _n;         // number of degrees of freedom of the robot
    int                             _k;         // number of constraints
    
    Eigen::MatrixRXd            _M;                      // floating-base mass matrix (n+6)x(n+6)
    Eigen::VectorXd             _h;                      // generalized bias forces (n+6)
    
    Eigen::MatrixRXd            _X;             /// matrix mapping constraint forces into momentum derivative
    Eigen::MatrixRXd            _Jc;            /// constraint Jacobian
    Eigen::VectorXd             _dJcdq;         /// dJc*dq
    Eigen::VectorXd             _fcDes;         /// desired constraint forces (result of QP)
    Eigen::Vector6d             _momentumDes;   /// desired momentum
    Eigen::VectorXd             _ddqDes;        /// desired accelerations (n+6)
    
    struct
    {
        Eigen::MatrixXd H;      /// Hessian
        Eigen::VectorXd g;      /// gradient
        Eigen::MatrixXd CE;
        Eigen::VectorXd ce0;
        Eigen::MatrixXd CI;
        Eigen::VectorXd ci0;
    } _qpData;
    
public:
    wbiStackOfTasks(wbi::wholeBodyInterface* robot);
    
    /** Update all tasks/constraints and compute the control
      * torques to send to the motors.
      * @param robotState The current state of the robot
      * @param torques Output control torques.
      */
    virtual void computeSolution(RobotState& robotState, Eigen::VectorRef torques);
    
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
    
};
    
    
    
    

/** Model of a task in the form of a quadratic cost function ||A*x-b||^2. */
class HQP_Task
{
public:
    Eigen::MatrixRXd A, Apinv, ApinvD;
    Eigen::MatrixRXd Spinv, SpinvD;
    Eigen::MatrixRXd N;     // nullspace projector of this task
    Eigen::VectorXd b;     // known term
    Eigen::VectorXd svA;   // singular value of the matrix A

    /** Create a task with the specified dimensions.
      * @param m Number of rows of A.
      * @param n Number of columns of A. */
    HQP_Task(int m, int n){ resize(m,n); }
    HQP_Task(){}
    void resize(int m, int n);

};

/** Solver for the following hierarchy of tasks:
  * - Task 1: foot constraints (either 1 foot or 2 feet)
  * - Task 2: center of mass (projection on the ground)
  * - Task 3: swinging foot
  * - Task 4: joint posture
  */
class WholeBodyReachSolver
{
    int n;                              // Number of joints (floating base included)
    Eigen::MatrixRXd     S;              // selection matrix for joints in the active set
    std::vector<int>    blockedJoints;  // list of blocked joints

    void blockJoint(int j);             // block the specified joint, adding it to the active set
public:
    HQP_Task        constraints;        // k DoFs
    HQP_Task        com;                // 2 DoFs
    HQP_Task        foot;               // 6 DoFs
    HQP_Task        posture;            // n-6 DoFs
    Eigen::VectorXd qMax;               // joint upper bounds (deg)
    Eigen::VectorXd qMin;               // joint lower bounds (deg)
    double          pinvTol;            // Tolerance used for computing truncated pseudoinverses
    double          pinvDamp;           // Damping factor used for computing damped pseudoinverses
    double          safetyThreshold;    // minimum distance from the joint bounds (deg)
    int             solverIterations;   // number of iterations required by the solver
    double          solverTime;         // time taken to compute the solution

    /** @param _k Number of constraints.
      * @param _n Number of joints (floating base included). 
      * @param _pinvTol Tolerance used for computing truncated pseudoinverses.
      * @param _pinvDamp Damping factor used for computing damped pseudoinverses. 
      * @param _safetyThreshold Minimum distance to maintain from the joint bounds.
      * @note The number of joints and constraints can be changed by calling the resize method.
      *       The tolerances can be changed as well by writing the corresponding public member variables.*/
    WholeBodyReachSolver(int _k, int _n, double _pinvTol, double _pinvDamp, double _safetyThreshold=0.0);

    /** Call this method any time either the number of joints or of constraints changes.
      * It resizes all the vectors and matrices.
      * @param _k Number of constraints.
      * @param _n Number of joints. */
    void resize(int _k, int _n);
    
    /** Find the desired joint velocities for solving the hierarchy of tasks.
      * @param qDes Output vector, desired joint velocities (rad/sec).
      * @param q Input vector, current joint positions (deg), used to check whether the joints are close to their bounds.
      */
    void solve(Eigen::VectorXd &dqDes, const Eigen::VectorXd &q);

    const std::vector<int>& getBlockedJointList(){ return blockedJoints; }
};

/** Tolerance for considering two values equal */
const double ZERO_TOL = 1e-5;

/**
 * Given the real position/orientation and the desired position/orientation, compute the error as a linear/angular velocity.
 * @param x Real position/orientation as 7d vector
 * @param xd Desired position/orientation as a 7d vector
 * @return w Output 6d vector, linear/angular velocity
 */
yarp::sig::Vector compute6DError(const yarp::sig::Vector &x, const yarp::sig::Vector &xd);

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

void assertEqual(const Eigen::MatrixRXd &A, const Eigen::MatrixRXd &B, std::string testName, double tol = ZERO_TOL);

void testFailed(std::string testName);

/** Convert a generic variable into a string. */
template <class T> inline std::string toString(const T& t)
{ std::ostringstream ss; ss << t; return ss.str(); }

/** Convert a generic vector into a string */
template <class T> inline std::string toString(const std::vector<T>& v, const char *separator=" ")
{ std::ostringstream s; std::copy(v.begin(), v.end(), std::ostream_iterator<T>(s, separator)); return s.str(); }

std::string toString(const Eigen::MatrixRXd &m, int precision=2, const char* endRowStr="\n", int maxRowsPerLine=10);

}

#endif
