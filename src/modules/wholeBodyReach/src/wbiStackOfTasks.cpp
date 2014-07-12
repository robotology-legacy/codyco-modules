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

wbiStackOfTasks::wbiStackOfTasks(wbi::wholeBodyInterface* robot, bool useNullspaceBase)
:   _robot(robot),
    _momentumTask(NULL),
    _postureTask(NULL),
    _jointLimitTask(NULL),
    _useNullspaceBase(useNullspaceBase)
{
    _qpData.activeSetSize = 0;
    this->useNullspaceBase(_useNullspaceBase==1);
    
    _n = robot->getDoFs();
    _M.resize(_n+6,_n+6);
    _h.resize(_n+6);
    
    _Mb_inv.setZero(6,6);
    _Mb_llt = LLT<MatrixRXd>(6);
    _ddqDes.setZero(_n+6);
    _ddq_jDes.setZero(_n);
    _ddq_jPosture.setZero(_n);

    _qpData.CE.resize(0,0);
    _qpData.ce0.resize(0);
    _qpData.CI.resize(0, 0);
    _qpData.ci0.resize(0);
}

bool wbiStackOfTasks::computeSolution(RobotState& robotState, Eigen::VectorRef torques)
{
    START_PROFILING(PROFILE_WHOLE_SOLVER);
    
    assert(_momentumTask!=NULL);
    assert(_postureTask!=NULL);
    assert(_jointLimitTask!=NULL);
    
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
    
    //*********************************
    // COMPUTE DESIRED CONTACT FORCES
    //*********************************
    START_PROFILING(PROFILE_FORCE_QP_PREP);
    {
        int index_k = 0, k=0, index_in=0, in=0;
        for(list<ContactConstraint*>::iterator it=_constraints.begin(); it!=_constraints.end(); it++)
        {
            ContactConstraint& c = **it;
            c.update(robotState);
            
            k  = c.getSize();    // number of constraint forces
            in = c.getNumberOfInequalities();

            c.getInequalityMatrix( _qpData.CI.block(index_in, index_k, in, k)); // CI = [CI, t.getInequalityMatrix()]
            c.getInequalityVector( _qpData.ci0.segment(index_in, in) );         //  b = [b; t.getInequalityVector()]
            c.getMomentumMapping(  _X.middleCols(index_k, k)  );                //  X = [X, t.getMomentumMapping()]
            c.getEqualityMatrix(   _Jc.middleRows(index_k, k) );                // Jc = [Jc; t.getEqualityMatrix()]
            c.getEqualityVector(   _dJcdq.segment(index_k, k) );                // dJc_dq = [dJc_dq; t.getEqualityVector()]
            
//            sendMsg("CI block "+c.getName()+":\n"+toString(_qpData.CI.block(index_in, index_k, in, k),1,"\n",12));
//            sendMsg("ci0: "+toString(_qpData.ci0.segment(index_in, in),1));

            index_k += k;
            index_in += in;
        }
//        sendMsg("CI:\n"+toString(_qpData.CI,1,"\n",12));
//        sendMsg("ci0: "+toString(_qpData.ci0,1));
        
        START_PROFILING(PROFILE_FORCE_QP_MOMENTUM);
        {
            _momentumTask->update(robotState);
            _momentumTask->getEqualityVector(_momentumDes);
        
            // @todo Check if possible to avoid this matrix-matrix multiplication
            _qpData.H   = _X.transpose()*_X + 1e-6*MatrixXd::Identity(_k,_k);
            _qpData.g   = -_X.transpose()*_momentumDes;
        }
        STOP_PROFILING(PROFILE_FORCE_QP_MOMENTUM);
    }
    STOP_PROFILING(PROFILE_FORCE_QP_PREP);
    
    sendMsg("momentumDes "+toString(_momentumDes,1));
//    cout<< "QP gradient "<<toString(_qpData.g,1)<<endl;
    double res;
    START_PROFILING(PROFILE_FORCE_QP);
    {
        res = solve_quadprog(_qpData.H, _qpData.g, _qpData.CE.transpose(), _qpData.ce0,
                             - _qpData.CI.transpose(), _qpData.ci0, _fcDes,
                             _qpData.activeSet, _qpData.activeSetSize);
    }
    STOP_PROFILING(PROFILE_FORCE_QP);
    if(res == std::numeric_limits<double>::infinity())
        return false;
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
        _ddq_jDes    = svdSolveWithDamping(_Jc_Sbar_svd, -_dJcdq-Jc_b*ddqDes_b, _numericalDamping);
//        _ddq_jDes    = _Jc_Sbar_svd.solve(-_dJcdq-Jc_b*ddqDes_b);
        ddqDes_b    -= _Mb_inv_M_bj*_ddq_jDes;
        ddqDes_j    += _ddq_jDes;
    }
    STOP_PROFILING(PROFILE_DDQ_CONTACT_CONSTR);
    
//    sendMsg("ddqDes (contact consistent) = "+toString(_ddqDes,1));
//    sendMsg("Base dynamics error  = "+toString((M_u*_ddqDes+h_b-Jc_b.transpose()*_fcDes).norm()));
//    sendMsg("Contact constr error = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
    
    _Z.setIdentity(_n,_n);
    updateNullspace(_Jc_Sbar_svd);
//    sendMsg("Jc*Sbar*Z = "+toString((_Jc_Sbar*_Z).sum()));
    
    // Solve the equality motion task
    // N=I
    // for i=1:K
    //      A = A_i*Sbar*N
    //      ddq_j = A^+ (b_i - A_i*ddq)
    //      ddq  += Sbar*ddq_j
    //      N    -= A^+ A
    // end
    if(false)
    {
        list<MinJerkPDLinkPoseTask*>::iterator it;
        for(it=_equalityTasks.begin(); it!=_equalityTasks.end(); it++)
        {
            MinJerkPDLinkPoseTask& t = **it;
            t.update(robotState);
            
            t.getEqualityMatrix(_A_i);
            t.getEqualityVector(_b_i);
            _A = _A_i.leftCols<6>()*_Mb_inv_M_bj + _A_i.rightCols(_n);
            _A *= _Z;
            _A_svd.compute(_A, _svdOptions);
            _ddq_jDes   = svdSolveWithDamping(_A_svd, _b_i - _A_i*_ddqDes, _numericalDamping); // @todo check this is not +=
            ddqDes_b    -= _Mb_inv_M_bj*_ddq_jDes;
            ddqDes_j    += _ddq_jDes;
            updateNullspace(_A_svd);
        }
    }
    
    // Now we can go on with the motion tasks, using the nullspace of dynamics and contacts:
    //      Z = nullspace(Jc*Sbar);
    START_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    {
        _postureTask->update(robotState);
        _postureTask->getEqualityVector(_ddq_jPosture);
        if(_useNullspaceBase)
            _ddq_jDes   = _Z * (_Z.transpose() * _ddq_jPosture);
        else
            _ddq_jDes   = _Z * _ddq_jPosture;
        ddqDes_b    -= _Mb_inv_M_bj*_ddq_jDes;
        ddqDes_j    += _ddq_jDes;
    }
    STOP_PROFILING(PROFILE_DDQ_POSTURE_TASK);
    
    torques     = M_a*_ddqDes + h_j - Jc_j.transpose()*_fcDes;
    
    STOP_PROFILING(PROFILE_WHOLE_SOLVER);
    
    sendMsg("fcDes        = "+toString(_fcDes,1));
    sendMsg("ddqDes       = "+toString(_ddqDes,1));
//    sendMsg("ddq_jDes     = "+toString(_ddq_jDes,1));
    sendMsg("ddq_jPosture = "+toString(_ddq_jPosture,1));
//    sendMsg("Base dynamics error  = "+toString((M_u*_ddqDes+h_b-Jc_b.transpose()*_fcDes).norm()));
//    sendMsg("Joint dynamics error = "+toString((M_a*_ddqDes+h_j-Jc_j.transpose()*_fcDes-torques).norm()));
//    sendMsg("Contact constr error = "+toString((_Jc*_ddqDes+_dJcdq).norm()));
    
//#define DEBUG_SOLVER
#ifdef DEBUG_SOLVER
    MatrixRXd Nc        = nullSpaceProjector(_Jc, PINV_TOL);
    MatrixRXd NcSTpinvD = pinvDampedEigen(Nc.rightCols(_n), _numericalDamping);
    MatrixRXd Jcpinv    = pinvDampedEigen(_Jc, _numericalDamping);
    VectorXd ddqDes1    = -Jcpinv*_dJcdq;
    VectorXd ddqDes     = ddqDes1 + NcSTpinvD.transpose()*(_ddq_jPosture - ddqDes1.tail(_n));
    VectorXd tauDes     = NcSTpinvD * (_M*ddqDes + _h);
//    sendMsg("-Jcpinv*_dJcdq = "+toString(ddqDes1,1));
//    sendMsg("ddqDes1 + NcSTpinvD^T*(_ddq_jPosture - ddqDes1) = "+toString(ddqDes,1));
//    sendMsg("(_M*ddqDes + _h)= "+toString((_M*ddqDes + _h),1));
    sendMsg("torques CFC   = "+toString(torques,1));
    sendMsg("tauDes (JSID) = "+toString(tauDes,1));
//    torques = tauDes;
#endif
    
    return true;
}

void wbiStackOfTasks::updateNullspace(JacobiSVD<MatrixRXd>& svd)
{
    int r = (svd.singularValues().array()>PINV_TOL).count();
    if(_useNullspaceBase)
        _Z *= svd.matrixV().rightCols(svd.cols()-r);
    else
        _Z -= svd.matrixV().leftCols(r)*svd.matrixV().leftCols(r).transpose();
}

void wbiStackOfTasks::computeMb_inverse()
{
    // @todo Improve this computation exploting structure of Mb
    _Mb_llt.compute(M_b);
    _Mb_inv.setIdentity(6,6);
    _Mb_llt.solveInPlace(_Mb_inv);
}

void wbiStackOfTasks::init(RobotState& robotState)
{
    _postureTask->init(robotState);
    _momentumTask->init(robotState);
    for(list<ContactConstraint*>::iterator it=_constraints.begin(); it!=_constraints.end(); it++)
        (*it)->init(robotState);
    for(list<MinJerkPDLinkPoseTask*>::iterator it=_equalityTasks.begin(); it!=_equalityTasks.end(); it++)
        (*it)->init(robotState);
        
}

void wbiStackOfTasks::addConstraint(ContactConstraint& constraint)
{
    _constraints.push_back(&constraint);
    _k = 0;
    int n_in=0;
    for(list<ContactConstraint*>::iterator it=_constraints.begin(); it!=_constraints.end(); it++)
    {
        _k   += (*it)->getSize();
        n_in += (*it)->getNumberOfInequalities();
    }
    _X.setZero(6,_k);
    _Jc.setZero(_k,_n+6);
    _dJcdq.setZero(_k);
    _fcDes.setZero(_k);
    _qpData.CI.setZero(n_in, _k);
    _qpData.ci0.setZero(n_in);
    _qpData.activeSet = VectorXi::Constant(n_in, -1);
}


void wbiStackOfTasks::linkParameterToVariable(ParamTypeId paramType, ParamHelperServer* paramHelper, int paramId)
{
    switch(paramType)
    {
        case USE_NULLSPACE_BASE:
            _useNullspaceBase_paramId = paramId;
            paramHelper->linkParam(paramId, &_useNullspaceBase);
            paramHelper->registerParamValueChangedCallback(paramId, this);
            break;
        case NUMERICAL_DAMPING:
            paramHelper->linkParam(paramId, &_numericalDamping);
            break;
        default:
            cout<< "[wbiStackOfTasks::linkParameterToVariable] Trying to link a parameter that is not managed\n";
    }
}

void wbiStackOfTasks::parameterUpdated(const ParamProxyInterface* pp)
{
    if(pp->id==_useNullspaceBase_paramId)
    {
        useNullspaceBase(_useNullspaceBase==1);
        return;
    }
    cout<< "[wbiStackOfTasks::parameterUpdated] Callback for a parameter that is not managed"
        << pp->name<< endl;
}

VectorXd wholeBodyReach::svdSolveWithDamping(const JacobiSVD<MatrixRXd>& A, VectorConst b, double damping)
{
    eigen_assert(A.computeU() && A.computeV() && "svdSolveWithDamping() requires both unitaries U and V to be computed (thin unitaries suffice).");
    assert(A.rows()==b.size());

//    cout<<"b = "<<toString(b,1)<<endl;
    VectorXd tmp(A.cols());
    int nzsv = A.nonzeroSingularValues();
    tmp.noalias() = A.matrixU().leftCols(nzsv).adjoint() * b;
//    cout<<"U^T*b = "<<toString(tmp,1)<<endl;
    double sv, d2 = damping*damping;
    for(int i=0; i<nzsv; i++)
    {
        sv = A.singularValues()(i);
        tmp(i) *= sv/(sv*sv + d2);
    }
//    cout<<"S^+ U^T b = "<<toString(tmp,1)<<endl;
    VectorXd res = A.matrixV().leftCols(nzsv) * tmp;
    
//    getLogger().sendMsg("sing val = "+toString(A.singularValues(),3), MSG_STREAM_INFO);
//    getLogger().sendMsg("solution with damp "+toString(damping)+" = "+toString(res.norm()), MSG_STREAM_INFO);
//    getLogger().sendMsg("solution without damping  ="+toString(A.solve(b).norm()), MSG_STREAM_INFO);
    
    return res;
}


//*************************************************************************************************************************
//************************************************* OLD STUFF *************************************************************
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
