/*
 * Copyright (C) 2014 Koroibot
 * Author: Andrea Del Prete
 * email:  adelpret@laas.fr
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

#include <iostream>
#include <iomanip>      // std::setprecision
#include <Eigen/Geometry>
#include <wholeBodyReach/Logger.h>
#include "wholeBodyReach/constrainedDynamicsIntegrator.h"

using namespace std;
using namespace Eigen;
using namespace wbi;
using namespace wholeBodyReach;
using namespace paramHelp;

ConstrainedDynamicsIntegrator::ConstrainedDynamicsIntegrator(wbi::iWholeBodyModel* robot)
: _p_B(_xB.p),
  _R_B(_xB.R.data)
{
    _time = 0.0;
    _robot = robot;
    _n = robot->getDoFs();
    _m = 0;
    _M.setZero(_n+6,_n+6);      /// mass matrix
    _h.setZero(_n+6);           /// bias forces
    _tau_np6.setZero(_n+6);     /// S^T*tau
    _ddq.setZero(_n+6);         /// joint+base accelerations
    _g(0) = _g(1) = 0.0;
    _g(2) = -9.81;

    _qj.setZero(_n);
    _dq.setZero(_n+6);

    int dim = 3+9+_n+_n+6;
    _x.setZero(dim);
    _x_i.setZero(dim);
    _dx.setZero(dim);
    _k1.setZero(dim);
    _k2.setZero(dim);
    _k3.setZero(dim);
    _k4.setZero(dim);

    _numericalDamping = 0.0;
    _timestep = 1e-4;
}

/*************************************************************************************************/
bool ConstrainedDynamicsIntegrator::addConstraints(string linkName)
{
    int linkId;
    if(_robot->getFrameList().idToIndex(linkName.c_str(), linkId)==false)
        return false;

    _m += 6;
    _constrainedLinkIds.push_back(linkId);
    resizeConstraintDataStructures();
    return true;
}

//**************************************************************************************************
void ConstrainedDynamicsIntegrator::linkParameterToVariable(ParamTypeId paramType,
                                                            ParamHelperServer* paramHelper,
                                                            int paramId)
{
    switch(paramType)
    {
        case CONSTR_NUM_DAMP:
            paramHelper->linkParam(paramId, &_numericalDamping);
            break;
        case TIMESTEP:
            paramHelper->linkParam(paramId, &_timestep);
            break;
        default:
            cout<< "[ConstrainedDynamicsIntegrator::linkParameterToVariable] Trying to link a parameter that is not managed\n";
    }
}

/*************************************************************************************************/
bool ConstrainedDynamicsIntegrator::integrate(VectorConst torques,
                                              const Frame& xB_i, VectorConst qj_i, VectorConst dq_i,
                                              Frame& xB_f,       VectorRef qj_f,   VectorRef dq_f)
{
    return integrate(_timestep, torques, xB_i, qj_i, dq_i, xB_f, qj_f, dq_f);
}

/*************************************************************************************************/
bool ConstrainedDynamicsIntegrator::integrate(double dt, VectorConst torques,
                                              const Frame& xB_i, VectorConst qj_i, VectorConst dq_i,
                                              Frame& xB_f,       VectorRef qj_f,   VectorRef dq_f)
{
    _time += dt;    /// integrate time

    // convert the state representation from q to x
    _tau = torques;
    _xB_i = xB_i;
    _xB   = xB_i;
    _x_i.head<3>()            = _p_B;
    MatrixR3d::Map(&_x_i[3])  = _R_B;
    _x_i.segment(12,_n)       = qj_i;
    _x_i.tail(_n+6)           = dq_i;

//    Frame H0_i, H1_i;
//    _robot->computeH(_x_i.segment(12,_n).data(), xB_i, _constrainedLinkIds[0], H0_i);
//    _robot->computeH(_x_i.segment(12,_n).data(), xB_i, _constrainedLinkIds[1], H1_i);

    // INTEGRATION
    //k1
    dynamics(_x_i, _dx);

    _ddq_first_call = _ddq;

#define USE_RK4
#ifdef USE_RK4
    _k1      = dt * _dx;
    _x      = _x_i + 0.5*_k1;
//    getLogger().sendMsg("R:  "+toString(_x_i.segment<9>(3),1), MSG_INFO);
//    getLogger().sendMsg("w:  "+toString(_x_i.segment<3>(12+_n+3),1), MSG_INFO);
//    getLogger().sendMsg("dR: "+toString(_dx.segment<9>(3),1), MSG_INFO);
    //k2
    dynamics(_x, _dx);
    _k2      = dt * _dx;
    _x      = _x_i + 0.5*_k2;
    //k3
    dynamics(_x, _dx);
    _k3      = dt * _dx;
    _x      = _x_i + _k3;
    //k4 and result
    dynamics(_x, _dx);
    _k4      = dt * _dx;
    double a6 = 1.0/6.0;
    double a3 = 1.0/3.0;
    _x       = _x_i + a6*_k1 + a3*_k2 + a3*_k3 + a6*_k4;
#else
    _x      = _x_i + dt*_dx;
#endif

    // convert from x to q
    _p_B    = _x.head<3>();
    _R_B    = MatrixR3d::Map(&_x[3]);
    xB_f    = _xB;
    qj_f     = _x.segment(12, _n);
    dq_f     = _x.tail(_n+6);

//    Frame H0_f, H1_f;
//    _robot->computeH(qj_f.data(), xB_f, _constrainedLinkIds[0], H0_f);
//    _robot->computeH(qj_f.data(), xB_f, _constrainedLinkIds[1], H1_f);

//    cout<<"qJ initial: "<<(WBR_RAD2DEG*qj_i.transpose())<<endl;
//    cout<<"qJ final:   "<<(WBR_RAD2DEG*qj_f.transpose())<<endl;
//    getLogger().sendMsg("Base initial:\n"+xB_i.toString(), MSG_STREAM_INFO);
//    cout<<"Base final:\n"<<xB_f.toString()<<endl;
//    getLogger().sendMsg("Constraint 0 initial:\n"+H0_i.toString(), MSG_STREAM_INFO);
//    cout<<"Constraint 0 final:\n"<<H0_f.toString()<<endl;
//    getLogger().sendMsg("Constraint 1 initial:\n"+H1_i.toString(), MSG_STREAM_INFO);
//    cout<<"Constraint 1 final:\n"<<H1_f.toString()<<endl;

    return true;
}

/*************************************************************************************************/
void ConstrainedDynamicsIntegrator::dynamics(VectorConst x, VectorRef dx)
{
    _p_B    = x.head<3>();
    _R_B    = MatrixR3d::Map(x.segment<9>(3).data());
    _qj     = x.segment(12, _n);
    _dq     = x.tail(_n+6);

    constrainedForwardDynamics(_tau, _xB, _qj, _dq, _ddq);

//    getLogger().sendMsg("tau integration: "+jointToString(_tau,1), MSG_STREAM_INFO);
//    getLogger().sendMsg("q integration: "+jointToString(WBR_RAD2DEG*_qj,1), MSG_STREAM_INFO);
//    getLogger().sendMsg("dq integration: "+jointToString(WBR_RAD2DEG*_dq,1), MSG_STREAM_INFO);
//    getLogger().sendMsg("xB integration:\n"+_xB.toString(), MSG_STREAM_INFO);

    Map<MatrixR3d>   dR(&dx[3]);
    dx.head<3>()        = _dq.head<3>();
    dR.col(0)           = _dq.segment<3>(3).cross(_R_B.col(0)); // dR = w x R
    dR.col(1)           = _dq.segment<3>(3).cross(_R_B.col(1)); // dR = w x R
    dR.col(2)           = _dq.segment<3>(3).cross(_R_B.col(2)); // dR = w x R
    dx.segment(12,_n)   = _dq.tail(_n);
    dx.tail(_n+6)       = _ddq;
}

/*************************************************************************************************/
void ConstrainedDynamicsIntegrator::constrainedForwardDynamics(Eigen::VectorConst torques, wbi::Frame &xBase,
                                                               Eigen::VectorRef qj, Eigen::VectorRef dq,
                                                               Eigen::VectorRef ddq)
{
    // compute constraint matrix and vector: A*ddq = b
    for(int i=0; i<_constrainedLinkIds.size(); i++)
    {
        _robot->computeJacobian(qj.data(), xBase, _constrainedLinkIds[i], _A.row(i*6).data());
        _robot->computeDJdq(qj.data(), xBase, dq.tail(_n).data(), dq.data(), _constrainedLinkIds[i], &_b[i*6]);
    }
    _b *= -1.0;

    // compute mass matrix and generalized bias forces
    _robot->computeMassMatrix(qj.data(), xBase, _M.data());
    _robot->computeGeneralizedBiasForces(qj.data(), xBase, dq.tail(_n).data(), dq.data(), _g.data(), _h.data());

    // compute constraint solution: ddqBar = - Jc^+ * dJc * dq
    _A_svd.compute(_A, ComputeFullU | ComputeFullV);
    _ddqBar = svdSolveWithDamping(_A_svd, _b, _numericalDamping);

    // compute base of null space of constraint Jacobian
    int r = (_A_svd.singularValues().array()>PINV_TOL).count();
    _Z = _A_svd.matrixV().rightCols(_n+6-r);

    // compute constrained accelerations ddq_c = (Z^T*M*Z)^{-1}*Z^T*(S^T*tau - h - M*ddqBar)
    _ZMZ = _Z.transpose()*_M*_Z;
    _ZMZ_chol.compute(_ZMZ);
    _tau_np6.tail(_n) = torques;
    _ddq_c = _Z.transpose()*(_tau_np6 - _h - _M*_ddqBar);
    _ZMZ_chol.solveInPlace(_ddq_c);

    // compute joint accelerations
    ddq = _ddqBar + _Z*_ddq_c;

//    getLogger().sendMsg("rank Jc = "+toString(r), MSG_STREAM_INFO);
//    getLogger().sendMsg("Z size = "+toString(_Z.rows())+" X "+toString(_Z.cols()), MSG_STREAM_INFO);
//    getLogger().sendMsg("A*ddq-b = "+toString((_A*ddq-_b).norm()), MSG_STREAM_INFO);
//    getLogger().sendMsg("ddqBar = "+jointToString(ddqBar,2), MSG_STREAM_INFO);
//    getLogger().sendMsg("ddq_c  = "+toString(_ddq_c,2), MSG_STREAM_INFO);
//    getLogger().sendMsg("ddq_c RHS = "+toString(_Z.transpose()*(_tau_np6 - _h - _M*_ddqBar),2), MSG_STREAM_INFO);
//    getLogger().sendMsg("Nc*(M*ddq+h-S^T*tau) = "+toString((_Z.transpose()*(_M*ddq+_h-_tau_np6)).norm()), MSG_STREAM_INFO);
//    getLogger().sendMsg("dJc*dq = "+toString(-1.0*_b,2), MSG_STREAM_INFO);
//    getLogger().sendMsg("Jc^T*f = "+toString(_M*ddq+_h-_tau_np6, 1), MSG_STREAM_INFO);
//    getLogger().sendMsg("ddq integration: "+jointToString(WBR_RAD2DEG*ddq,2), MSG_STREAM_INFO);
//    getLogger().sendMsg("Jc:\n"+toString(_A,1), MSG_STREAM_INFO);
//    getLogger().sendMsg("Z:\n"+toString(_Z.transpose(),2,"\n",16), MSG_STREAM_INFO);
}

/*************************************************************************************************/
void ConstrainedDynamicsIntegrator::resizeConstraintDataStructures()
{
    _A.setZero(_m,_n+6);        /// constraint matrix
    _b.setZero(_m);             /// constraint vector
    _A_svd = SVD(_m,_n+6);      /// svd of the constraint matrix

    int k = _n+6-_m;            /// (likely) dimension of null space of the constraints
    if(k<0) k=0;
    _Z.setZero(_n+6,k);
    _ZMZ.setZero(k,k);          /// projected mass matrix: Z_c^T*M*Z_c
    _ZMZ_chol = Cholesky(k);    /// Cholesky decomposition of _ZMZ
    _ddq_c.setZero(k);          /// constrained accelerations
}

