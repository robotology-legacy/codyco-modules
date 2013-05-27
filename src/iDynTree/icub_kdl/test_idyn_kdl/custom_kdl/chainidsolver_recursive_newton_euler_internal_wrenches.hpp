// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef KDL_CHAIN_IKSOLVER_RECURSIVE_NEWTON_EULER_INTERNAL_WRENCHES_HPP
#define KDL_CHAIN_IKSOLVER_RECURSIVE_NEWTON_EULER_INTERNAL_WRENCHES_HPP

#include <kdl/chainidsolver.hpp>

namespace KDL{
    /**
     * \brief Recursive newton euler inverse dynamics solver
     *
     * The algorithm implementation is based on the book "Rigid Body
     * Dynamics Algorithms" of Roy Featherstone, 2008
     * (ISBN:978-0-387-74314-1) See page 96 for the pseudo-code.
     * 
     * It calculates the torques for the joints, given the motion of
     * the joints (q,qdot,qdotdot), external forces on the segments
     * (expressed in the segments reference frame) and the dynamical
     * parameters of the segments.
     */
    class ChainIdSolver_RNE_IW : public ChainIdSolver{
    public:
        /**
         * Constructor for the solver, it will allocate all the necessary memory
         * \param chain The kinematic chain to calculate the inverse dynamics for, an internal copy will be made.
         * \param grav The gravity vector to use during the calculation.
         */
        ChainIdSolver_RNE_IW(const Chain& chain,Vector grav);
        ~ChainIdSolver_RNE_IW(){};
        

        int CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques) { } ;
        /**
         * Function to calculate from Cartesian forces to joint torques.
         * Input parameters;
         * \param q The current joint positions
         * \param q_dot The current joint velocities
         * \param q_dotdot The current joint accelerations
         * \param f_ext The external forces (no gravity) on the segments
         * Output parameters:
         * \param torques the resulting torques for the joints
         * \param f_int The internal forces on the segments
         */
        int CartToJnt_and_internal_wrenches(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques,Wrenches& f_int);

    private:
        Chain chain;
        unsigned int nj;
        unsigned int ns;
        std::vector<Frame> X;
        std::vector<Twist> S;
        std::vector<Twist> v;
        std::vector<Twist> a;
        std::vector<Wrench> f;
        Twist ag;
    };
}

#endif
