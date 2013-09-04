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

#ifndef EQUALITY_QP
#define EQUALITY_QP

#include <Eigen/Core>               // import most common Eigen types
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>           // Timer
#include <vector>
#include <string>
#include <iostream>
#include <iterator>

namespace locomotion
{

/** Model of a task in the form of a quadratic cost function ||A*x-b||^2. */
class HQP_Task
{
public:
    Eigen::MatrixXd A, Apinv, ApinvD;
    Eigen::MatrixXd Spinv, SpinvD;
    Eigen::MatrixXd N;     // nullspace projector of this task
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
class LocomotionSolver
{
    int n;                              // Number of joints (floating base included)
    Eigen::MatrixXd     S;              // selection matrix for joints in the active set
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
    LocomotionSolver(int _k, int _n, double _pinvTol, double _pinvDamp, double _safetyThreshold=0.0);

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
  * @param U Output mXm matrix, left eigenvectors of A. 
  * @param V Output nXn matrix, right eigenvectors of A. 
  * @param Spinv Output kXk matrix (with k=min(m,n)), truncated pseudoinverse of the singular value matrix of A. 
  * @param Apinv Output nXm matrix, truncated pseudoinverse of A.
  * @param sv Output (optional) k-dim vector (with k=min(m,n)), singular values of A. */
void pinvTrunc(const Eigen::MatrixXd &A, double tol, Eigen::MatrixXd &Apinv, Eigen::MatrixXd &Spinv, Eigen::VectorXd &sv);

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
void pinvDampTrunc(const Eigen::MatrixXd &A, double tol, double damp, Eigen::MatrixXd &Apinv, Eigen::MatrixXd &ApinvDamp, Eigen::MatrixXd &Spinv, Eigen::MatrixXd &SpinvD, Eigen::VectorXd &sv);

void assertEqual(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, std::string testName, double tol = ZERO_TOL);

void testFailed(std::string testName);

/** Convert a generic variable into a string. */
template <class T> inline std::string toString(const T& t)
{ std::ostringstream ss; ss << t; return ss.str(); }

/** Convert a generic vector into a string */
template <class T> inline std::string toString(const std::vector<T>& v, const char *separator=" ")
{ std::ostringstream s; std::copy(v.begin(), v.end(), std::ostream_iterator<T>(s, separator)); return s.str(); }

std::string toString(const Eigen::MatrixXd &m, int precision=2, const char* endRowStr="\n", int maxRowsPerLine=10);

}

#endif
