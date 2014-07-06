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
#include <wholeBodyReach/eiquadprog2011.hpp>
#include <wholeBodyReach/Stopwatch.h>
#include <Eigen/SVD>

#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/math.h>
#include <iostream>
#include <cstdio>

using namespace wholeBodyReach;
using namespace Eigen;
using namespace std;
using namespace iCub::ctrl;
using namespace yarp::math;


#define Jc_j        _Jc.rightCols(_n)
#define Jc_b        _Jc.leftCols<6>()
#define M_b         _M.topLeftCorner<6, 6>()
#define M_bj        _M.topRightCorner(6,_n)
#define M_a         _M.bottomRows(_n)
#define h_b         _h.head<6>()
#define h_j         _h.tail(_n)
#define ddqDes_b    _ddqDes.head<6>()
#define ddqDes_j    _ddqDes.tail(_n)

#define PROFILE_DYNAMICS_COMPUTATION    "Dynamics computation"
#define PROFILE_FORCE_QP_PREP           "Force QP preparation"
#define PROFILE_FORCE_QP                "Force QP"
#define PROFILE_FORCE_TOTAL             "Force overall computation"
#define PROFILE_DDQ_DYNAMICS_CONSTR     "Compute dynamics-consistent ddq"
#define PROFILE_DDQ_CONTACT_CONSTR      "Compute contact-consistent ddq"
#define PROFILE_DDQ_POSTURE_TASK        "Compute posture task"

wbiStackOfTasks::wbiStackOfTasks(wbi::wholeBodyInterface* robot)
:   _robot(robot),
    _momentumTask(NULL),
    _postureTask(NULL),
    _jointLimitTask(NULL),
    _useNullspaceBase(false)
{
    _svdOptions = _useNullspaceBase ? ComputeFullU|ComputeFullV : ComputeThinU|ComputeThinV;
    
    _n = robot->getDoFs();
    _M.resize(_n+6,_n+6);
    _h.resize(_n+6);
    
    _Mb_inv.setZero(6,6);
    _Mb_llt = LLT<MatrixRXd>(6);
//    _Jc_Sbar_svd.setThreshold(1e-4);
    _ddqDes.setZero(_n+6);
    _ddqPosture.setZero(_n+6);

    _qpData.CE.resize(0,0);
    _qpData.ce0.resize(0);
    _qpData.CI.resize(0, 0);
    _qpData.ci0.resize(0);
}

void wbiStackOfTasks::computeSolution(RobotState& robotState, Eigen::VectorRef torques)
{
    assert(_momentumTask!=NULL);
    assert(_postureTask!=NULL);
    assert(_jointLimitTask!=NULL);
    
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
    
    //*********************************
    // COMPUTE DESIRED CONTACT FORCES
    //*********************************
    START_PROFILING(PROFILE_FORCE_QP_PREP);
    {
        int index = 0;
        for(list<ContactConstraint*>::iterator it=_constraints.begin(); it!=_constraints.end(); it++)
        {
            ContactConstraint& c = **it;
            c.update(robotState);

    //        B = [B; t.getInequalityMatrix()]
    //        b = [b; t.getInequalityVector()]
            c.getMomentumMapping(   _X.block(       0,      index,  6, 6));
            c.getEqualityMatrix(    _Jc.block(      index,  0,      6, _n+6));
            c.getEqualityVector(    _dJcdq.segment( index,  6));

            index += c.getSize();
        }
        _momentumTask->update(robotState);
        _momentumTask->getEqualityVector(_momentumDes);
        
        // @todo Check if possible to avoid this matrix-matrix multiplication
        _qpData.H   = _X.transpose()*_X + 1e-4*MatrixXd::Identity(_k,_k);
        _qpData.g   = _X.transpose()*_momentumDes;
    }
    STOP_PROFILING(PROFILE_FORCE_QP_PREP);
    
    START_PROFILING(PROFILE_FORCE_QP);
    {
        solve_quadprog(_qpData.H, _qpData.g, _qpData.CE, _qpData.ce0, _qpData.CI, _qpData.ci0, _fcDes);
    }
    STOP_PROFILING(PROFILE_FORCE_QP);
    
    //*********************************
    // COMPUTE DESIRED ACCELERATIONS
    //*********************************
    
    // Compute ddq that respect dynamics:
    //     ddqBar = [Mb^{-1}*(Jc_u^T*f-h_u); zeros(n,1)];
    START_PROFILING(PROFILE_DDQ_DYNAMICS_CONSTR);
    {
        computeMb_inverse();
        ddqDes_b    = _Mb_inv*(Jc_b.transpose()*_fcDes - h_b);
    }
    STOP_PROFILING(PROFILE_DDQ_DYNAMICS_CONSTR);

    // Compute ddq that respect also contact constraints:
    //      Sbar = [-Mb^{-1}*Mbj; eye(n)];
    //      z = (Jc*Sbar).solve(-Jc*ddqBar - dJc_dq);
    //      ddqBar += Sbar*zBar
    START_PROFILING(PROFILE_DDQ_CONTACT_CONSTR);
    {
        _Jc_Sbar    = Jc_j - (Jc_b * _Mb_inv * M_bj);
        _Jc_Sbar_svd.compute(_Jc_Sbar, _svdOptions);
        _z          = -1.0 * _Jc_Sbar_svd.solve(Jc_b*ddqDes_b + _dJcdq);    // _z \in \R^{_n}
        ddqDes_b    += _Mb_inv*(M_bj*_z);
        ddqDes_j    += _z;
    }
    STOP_PROFILING(PROFILE_DDQ_CONTACT_CONSTR);
    
    // Now we can go on with the motion tasks, using the nullspace of dynamics and contacts:
    //      Z = Sbar;
    //      Z *= nullspaceBasis(Jc*Sbar);
    START_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    {
        _Z.resize(_n+6,_n);
        _Z.topRows<6>() = _Mb_inv * M_bj;
        _Z.bottomRows(_n).setIdentity();
        updateNullspace(_Jc_Sbar_svd);

        _postureTask->update(robotState);
        _postureTask->getEqualityVector(_ddqPosture);
        _ddqDes     += _Z * (_Z.transpose() * _ddqPosture);
    }
    STOP_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    
    torques     = M_a*_ddqDes + h_j - Jc_j.transpose()*_fcDes;
}

void wbiStackOfTasks::updateNullspace(JacobiSVD<MatrixRXd>& svd)
{
    int r = (svd.singularValues().array()>PINV_TOL).count();
    if(_useNullspaceBase)
        _Z *= svd.matrixV().rightCols(svd.cols()-r);
    else
        _Z -= svd.matrixV()*svd.matrixV().transpose();
}

void wbiStackOfTasks::computeMb_inverse()
{
    // @todo Improve this computation exploting structure of Mb
    _Mb_llt.compute(M_b);
    _Mb_inv.setIdentity(6,6);
    _Mb_llt.solveInPlace(_Mb_inv);
}


void wbiStackOfTasks::addConstraint(ContactConstraint& constraint)
{
    _constraints.push_back(&constraint);
    _k = 0;
    for(list<ContactConstraint*>::iterator it=_constraints.begin(); it!=_constraints.end(); it++)
        _k += (**it).getSize();
    _X.resize(6,_k);
    _Jc.resize(_k,_n+6);
    _dJcdq.resize(_k);
    _fcDes.resize(_k);
}


//*************************************************************************************************************************
//************************************************* OLD STUFF *************************************************************
//*************************************************************************************************************************

WholeBodyReachSolver::WholeBodyReachSolver(int _k, int _n, double _pinvTol, double _pinvDamp, double _st)
    : pinvTol(_pinvTol), pinvDamp(_pinvDamp), safetyThreshold(_st)
{
    resize(_k,_n);
}

//*************************************************************************************************************************
void WholeBodyReachSolver::resize(int _k, int _n)
{
    n = _n;
    constraints.resize(_k,n);
    com.resize(2,n);
    foot.resize(6,n);
    posture.resize(n-6,n);
    S.resize(n,n); S.setIdentity();
    qMax.resize(n-6);
    qMin.resize(n-6);
}

//*************************************************************************************************************************
void WholeBodyReachSolver::solve(VectorXd &dqjDes, const VectorXd &q)
{
    double t0 = yarp::os::Time::now();
    bool solutionFound=false;
    solverIterations = 0;
    VectorXd dqDes; 
    S.setIdentity();
    blockedJoints.resize(0);

    while(!solutionFound)
    {
        solverIterations++;
        dqDes = VectorXd::Zero(n);
        constraints.N.setIdentity();
        // *** CONTACT CONSTRAINTS
        pinvTrunc(constraints.A*S, pinvTol, constraints.Apinv, constraints.Spinv, constraints.svA);
        constraints.N -= constraints.Apinv*constraints.A*S;
        // *** COM CTRL TASK
        pinvDampTrunc(com.A*S*constraints.N, pinvTol, pinvDamp, com.Apinv, com.ApinvD, com.Spinv, com.SpinvD, com.svA);
        dqDes += com.ApinvD*com.b;
        com.N = constraints.N - com.Apinv*com.A*S*constraints.N;
        // *** FOOT CTRL TASK
        pinvDampTrunc(foot.A*S*com.N, pinvTol, pinvDamp, foot.Apinv, foot.ApinvD, foot.Spinv, foot.SpinvD, foot.svA);
        dqDes += foot.ApinvD*(foot.b - foot.A*S*dqDes);
        foot.N = com.N - foot.Apinv*foot.A*S*com.N;
        // *** POSTURE TASK
        pinvTrunc(posture.A*S*foot.N, pinvTol, posture.Apinv, posture.Spinv, posture.svA);
        dqDes += posture.Apinv*(posture.b - posture.A*S*dqDes);
    
        dqjDes = (S*dqDes).tail(n-6);  // return last n-6 joint vel (i.e. discard base vel)

        solutionFound = true;
        for(int i=0; i<n-6; i++)
        {
            if((q(i)+safetyThreshold>=qMax(i) && dqjDes(i)>0.0) || (q(i)-safetyThreshold<=qMin(i) && dqjDes(i)<0.0))  
            {
                blockJoint(i);          // add joint i to the active set
                solutionFound = false;
            }
        }
    }
    solverTime = yarp::os::Time::now()-t0;
}

//*************************************************************************************************************************
void WholeBodyReachSolver::blockJoint(int j)
{
    S(6+j,6+j) = 0.0;
    blockedJoints.push_back(j);
}

//*************************************************************************************************************************
void HQP_Task::resize(int m, int n)
{
    A.resize(m,n);
    Apinv.resize(n,m);
    ApinvD.resize(n,m);
    N.resize(n,n);
    svA.resize(m);
    b.resize(m);
}


//*************************************************************************************************************************
yarp::sig::Vector wholeBodyReach::compute6DError(const yarp::sig::Vector &x, const yarp::sig::Vector &xd)
{
    yarp::sig::Matrix R     = axis2dcm(x.subVector(3,6));
    yarp::sig::Matrix Rdes  = axis2dcm(xd.subVector(3,6));
    yarp::sig::Matrix Re    = Rdes * R.transposed();
    yarp::sig::Vector aa    = dcm2axis(Re);
    yarp::sig::Vector res(6);
    res[0] = xd[0]-x[0];
    res[1] = xd[1]-x[1];
    res[2] = xd[2]-x[2];
    res[3] = aa[3] * aa[0];
    res[4] = aa[3] * aa[1];
    res[5] = aa[3] * aa[2];
    return res;
}

//*************************************************************************************************************************
void wholeBodyReach::pinvTrunc(const MatrixRXd &A, double tol, MatrixRXd &Apinv, MatrixRXd &Spinv, VectorXd &sv)
{
    // allocate memory
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
    Spinv.setZero(k,k);
    // compute decomposition
    JacobiSVD<MatrixRXd> svd(A, ComputeThinU | ComputeThinV);    // default Eigen SVD
    sv = svd.singularValues();
    // compute pseudoinverse of singular value matrix
    for (int c=0;c<k; c++)
        if ( sv(c)> tol)
            Spinv(c,c) = 1/sv(c);
    // compute pseudoinverse
    Apinv = svd.matrixV() * Spinv  * svd.matrixU().transpose();
}


//*************************************************************************************************************************
void wholeBodyReach::pinvDampTrunc(const MatrixRXd &A, double tol, double damp, MatrixRXd &Apinv, MatrixRXd &ApinvDamp, MatrixRXd &Spinv, MatrixRXd &SpinvD, VectorXd &sv)
{
    // allocate memory
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
    Spinv.setZero(k,k); 
    SpinvD.setZero(k,k);
    // compute decomposition
    JacobiSVD<MatrixRXd> svd(A, ComputeThinU | ComputeThinV);    // default Eigen SVD
    sv = svd.singularValues();
    // compute pseudoinverses of singular value matrix
    double damp2 = damp*damp;
    for (int c=0;c<k; c++)
    {
        SpinvD(c,c) = sv(c) / (sv(c)*sv(c) + damp2);
        if ( sv(c)> tol)
            Spinv(c,c) = 1/sv(c);
    }
    // compute pseudoinverses
    Apinv       = svd.matrixV() * Spinv  * svd.matrixU().transpose();   // truncated pseudoinverse
    ApinvDamp   = svd.matrixV() * SpinvD * svd.matrixU().transpose();   // damped pseudoinverse
}

Eigen::MatrixRXd wholeBodyReach::pinvDampedEigen(const Eigen::Ref<Eigen::MatrixRXd> &A, double damp)
{
    // allocate memory
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
    VectorXd SpinvD = VectorXd::Zero(k);
    // compute decomposition
    JacobiSVD<MatrixRXd> svd(A, ComputeThinU | ComputeThinV);    // default Eigen SVD
    VectorXd sv = svd.singularValues();
    // compute pseudoinverse of singular value matrix
    double damp2 = damp*damp;
    for (int c=0;c<k; c++)
        SpinvD(c) = sv(c) / (sv(c)*sv(c) + damp2);
    // compute damped pseudoinverse
    return svd.matrixV() * SpinvD.asDiagonal() * svd.matrixU().transpose();
}

Eigen::MatrixRXd wholeBodyReach::nullSpaceProjector(const Eigen::Ref<MatrixRXd> A, double tol)
{
    // allocate memory
    int m = A.rows(), n = A.cols(), k = m<n?m:n;
    MatrixRXd Spinv = MatrixRXd::Zero(k,k);
    // compute decomposition
    JacobiSVD<MatrixRXd> svd(A, ComputeThinU | ComputeThinV);    // default Eigen SVD
    VectorXd sv = svd.singularValues();
    // compute pseudoinverse of singular value matrix
    for (int c=0;c<k; c++)
        if ( sv(c)> tol)
            Spinv(c,c) = 1/sv(c);
    // compute pseudoinverse
    MatrixRXd N = MatrixRXd::Identity(n,n);
    N -= svd.matrixV() * Spinv  * svd.matrixU().transpose() * A;
    return N;
}

//*************************************************************************************************************************
void wholeBodyReach::assertEqual(const MatrixRXd &A, const MatrixRXd &B, string testName, double tol)
{
    if(A.cols() != B.cols() || A.rows()!=B.rows())
    {
        cout<< testName<< ": dim(A) != dim(B): " << A.rows()<< "x"<<A.cols()<<" != "<< B.rows()<< "x"<<B.cols()<<endl;
        return testFailed(testName);
    }
    for(int r=0; r<A.rows(); r++)
        for(int c=0; c<A.cols(); c++)
            if(abs(A(r,c)-B(r,c))>tol)
            {
                printf("%s: element %d,%d is different, absolute difference is %f\n", testName.c_str(), r, c, abs(A(r,c)-B(r,c)));
                return testFailed(testName);
            }
}

//*************************************************************************************************************************
void wholeBodyReach::testFailed(string testName)
{
    printf("Test %s ***FAILED*** !!!\n", testName.c_str());
    assert(false);
}

//*************************************************************************************************************************
std::string wholeBodyReach::toString(const Eigen::MatrixRXd &m, int precision, const char* endRowStr, int maxColsPerLine)
{
    string ret = "";
    if(m.rows()>1 && m.cols()>maxColsPerLine)
    {
        return ret+"("+toString(maxColsPerLine)+" cols)\n" + 
            toString(m.leftCols(maxColsPerLine),precision,endRowStr,maxColsPerLine) + "\n" +
            toString(m.rightCols(m.cols()-maxColsPerLine),precision,endRowStr,maxColsPerLine);
    }
    char tmp[350];
    for(int i=0;i<m.rows();i++)
    {
        for(int j=0;j<m.cols();j++)
        {
            sprintf(tmp, "% .*lf\t", precision, m(i,j));
            ret+=tmp;
        }
        ret = ret.substr(0,ret.length()-1);     // remove the last character (tab)
        if(i<m.rows()-1)                          // if it is not the last row
            ret+= endRowStr;
    }
    return ret.substr(0, ret.length()-1);
}