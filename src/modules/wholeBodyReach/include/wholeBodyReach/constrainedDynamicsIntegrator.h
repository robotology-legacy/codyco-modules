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



#ifndef WBR_CONSTRAINED_DYNAMICS_INTEGRATOR_H
#define WBR_CONSTRAINED_DYNAMICS_INTEGRATOR_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <wholeBodyReach/wholeBodyReachConstants.h>
#include <wholeBodyReach/wholeBodyReachUtils.h>
#include <wbi/wholeBodyInterface.h>
#include <wbi/wbiUtil.h>
#include <paramHelp/paramHelperServer.h>

namespace wholeBodyReach
{

/** A class for integrating the dynamics of a floating-base robot
 * subject to rigid contact constraints. Currently, the only available
 * integration scheme is Runge-Kutte 4th order.
*/
class ConstrainedDynamicsIntegrator
{
public:
    /// This enum defines all the parameters of this class
    enum ParamTypeId
    {
        CONSTR_NUM_DAMP,       /// numerical damping used when solving dynamics
        TIMESTEP            /// timestep used for integration
    };
    
// this should return protected after debugging
public:
    wbi::iWholeBodyModel*   _robot;     /// interface to robot dynamics computation
    int                     _n;         /// number of joints
    int                     _m;         /// number of constraints
    double                  _numericalDamping;  /// numerical damping to regularize constraint resolution
    double                  _timestep;              /// default timestep used if none is specified
    std::vector<int>        _constrainedLinkIds;    /// list of ID of the constrained links

    Eigen::MatrixRXd        _A;         /// constraint matrix
    Eigen::VectorXd         _b;         /// constraint vector
    Eigen::SVD              _A_svd;     /// svd of the constraint matrix
    
    Eigen::MatrixRXd        _M;         /// mass matrix
    Eigen::VectorXd         _h;         /// bias forces
    Eigen::Vector3d         _g;         /// gravity acceleration
    Eigen::VectorXd         _tau;       /// joint torques
    Eigen::VectorXd         _tau_np6;   /// S^T*tau
    Eigen::VectorXd         _ddq;       /// joint+base acceleration
    
    Eigen::MatrixRXd        _Z;         /// base of constraint null space
    Eigen::MatrixRXd        _ZMZ;       /// projected mass matrix: Z_c^T*M*Z_c
    Eigen::Cholesky         _ZMZ_chol;  /// Cholesky decomposition of _ZMZ
    Eigen::VectorXd         _ddq_c;     /// constrained accelerations
    Eigen::VectorXd         _ddqBar;    /// solution of Jc*ddq = -dJc*dq
    
    Eigen::VectorXd                 _qj;    /// joint positions
    Eigen::VectorXd                 _dq;    /// base+joint velocities
    wbi::Frame                      _xB;    /// base pose
    Eigen::Map<Eigen::Vector3d>     _p_B;   /// (_xB.p);
    Eigen::Map<Eigen::MatrixR3d>    _R_B;   /// (_xB.R.data);  // row major data
    
    Eigen::VectorXd _x;         /// state of the integration
    Eigen::VectorXd _x_i;       /// initial state of the integration
    Eigen::VectorXd _dx;        /// state derivative of the integration
    Eigen::VectorXd _k1, _k2, _k3, _k4; /// temp variables used in integration
    
    void resizeConstraintDataStructures();
    
    /** Compute the constrained forward dynamics of the robot.
     * @param torques The joint torques
     * @param xBase The base position and orientation
     * @param q The joint positions
     * @param dq The joint and base velocities
     * @param ddq Output joint and base accelerations
     */
    void constrainedForwardDynamics(Eigen::VectorConst torques, wbi::Frame& xBase, Eigen::VectorRef qj,
                                    Eigen::VectorRef dq, Eigen::VectorRef ddq);
    
    /** Compute the dynamics of the constrained robot under the general form dx = f(x).
     * The state is represented as: 3 elements for base position, 9 for base orientation, 
     * n for joint positions, 6 elements for base velocity, n elements for joint velocities
     * @param x State of the robot.
     * @param dx Derivative of the state.
     */
    void dynamics(Eigen::VectorConst x, Eigen::VectorRef dx);
    
public:
    
	/** Constructor */
	ConstrainedDynamicsIntegrator(wbi::iWholeBodyModel* robot);
    
	/** Destructor */
	~ConstrainedDynamicsIntegrator(){};
    
    /** Add the specified link to the link of constrained links.
      * When integrating the robot's dynamics the constrained links are considered as subject to 
      * physical constraints that prevent their motion.
      * @param linkName Name of the constrained link.
      * @return True if the operation succeeded, false otherwise.
     */
    virtual bool addConstraints(std::string linkName);
    
    virtual void linkParameterToVariable(ParamTypeId paramType,
                                         paramHelp::ParamHelperServer* paramHelper,
                                         int paramId);

    /** Integrate the equations of motion of the floating-base robot using the specified parameters.
     * The system is supposed to be subject to a set of equality constraints on the accelerations
     * in the form: A*ddq = b.
     * @param timestep The length of the timestep to integrate in seconds.
     * @param torques The value of the joint torques (assumed to be constant during the whole timestep)
     * @param xB_i The initial value of the base position and orientation
     * @param qj_i The initial value of the joint positions
     * @param dq_i The initial value of the joint and base velocities
     * @param xB_f The output value of the base position and orientation at the end of the timestep
     * @param q_f The output value of the joint positions at the end of the timestep
     * @param dq_f The output value of the joint and base velocities at the end of the timestep
     * @return True if everything went fine, false otherwise.
     */
	virtual bool integrate(double timestep, Eigen::VectorConst torques,
                           const wbi::Frame& xB_i, Eigen::VectorConst qj_i, Eigen::VectorConst dq_i,
                           wbi::Frame& xB_f, Eigen::VectorRef qj_f, Eigen::VectorRef dq_f);
    
    virtual bool integrate(Eigen::VectorConst torques,
                           const wbi::Frame& xB_i, Eigen::VectorConst qj_i, Eigen::VectorConst dq_i,
                           wbi::Frame& xB_f, Eigen::VectorRef qj_f, Eigen::VectorRef dq_f);

};
    
};  // end namespace wholeBodyReach

#endif
