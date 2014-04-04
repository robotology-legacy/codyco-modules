/*
 * Copyright (C) 2013  CoDyCo Consortium
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
 *
 * Authors: Andrea Del Prete, Silvio Traversaro, Marco Randazzo
 * email: andrea.delprete@iit.it - silvio.traversaro@iit.it - marco.randazzo@iit.it
 */

#ifndef IWHOLEBODYMODEL_H
#define IWHOLEBODYMODEL_H

namespace wbi {
    class LocalId;
    class LocalIdList;
    class Frame;
    
    /**
     * Interface to the kinematic/dynamic model of the robot.
     */
    class iWholeBodyModel
    {
    public:
        /** Id of the virtual link associated to the Center of Mass of the robot.
         * This id can be used to compute any kinematic quantity (position, velocity, Jacobian)
         * associated to the center of mass. */
        static const int COM_LINK_ID = -1;
        
        virtual ~iWholeBodyModel();
        virtual bool init() = 0;
        virtual bool close() = 0;
        
        /** @return The number of degrees of freedom of the robot model. */
        virtual int getDoFs() = 0;
        
        /** Remove the specified joint from the robot model. The joint is considered blocked
         * at its last known value (values of the joint angles are stored whenever a method of
         * iWholeBodyModel is called). If no previous value of the joint angle is known, zero is assumed.
         * @param j Id of the joint to remove
         * @return True if the joint was found and removed, false otherwise. */
        virtual bool removeJoint(const LocalId &j) = 0;
        virtual bool addJoint(const LocalId &j) = 0;
        virtual int addJoints(const LocalIdList &j) = 0;
        virtual const LocalIdList& getJointList() = 0;
        
        /** Get the upper and lower limits of the joint position(s).
         * @param qMin Output lower limit(s) (rad).
         * @param qMax Output upper limit(s) (rad).
         * @param joint Id of the joint, -1 for getting the limits of all the joints.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool getJointLimits(double *qMin, double *qMax, int joint=-1) = 0;
        
        /** Get the id of the link with the specified name.
         * @param linkName Name of the link.
         * @param linkId Id of the link (if found).
         * @return True if the link name was found, false otherwise. */
        virtual bool getLinkId(const char *linkName, int &linkId) = 0;
        
        /** Compute rototranslation matrix from root reference frame to reference frame associated to the specified link.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame.
         * @param linkId Id of the link that is the target of the rototranslation.
         * @param H Output 4x4 rototranslation matrix (stored by rows).
         * @return True if the operation succeeded, false otherwise (invalid input parameters). */
        virtual bool computeH(double *q, const Frame &xBase, int linkId, Frame &H) = 0;
        
        /** Compute the Jacobian of the specified point of the robot.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame.
         * @param linkId Id of the link.
         * @param J Output 6xN Jacobian matrix (stored by rows), where N=number of joints.
         * @param pos 3d position of the point expressed w.r.t the link reference frame.
         * @return True if the operation succeeded, false otherwise (invalid input parameters).
         * @note If linkId==COM_LINK_ID then the angular part of J is related to the angular velocity of the
         *       whole multi-body system. This Jacobian premultiplied by the whole robot's 6D inertia
         *       matrix is equal to the Jacobian of the angular momentum of the whole robot. */
        virtual bool computeJacobian(double *q, const Frame &xBase, int linkId, double *J, double *pos=0) = 0;
        
        /** Given a point on the robot, compute the product between the time derivative of its
         * Jacobian and the joint velocity vector.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame.
         * @param dq Joint velocities (rad/s).
         * @param linkId Id of the link.
         * @param dJdq Output 6-dim vector containing the product \f$\dot{J}\dot{q}\f$.
         * @param pos 3d position of the point expressed w.r.t the link reference frame.
         * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeDJdq(double *q, const Frame &xBase, double *dq, double *dxB, int linkId, double *dJdq, double *pos=0) = 0;
        
        /** Compute the forward kinematics of the specified joint.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame
         * @param linkId Id of the link.
         * @param x Output Rototranslation from world frame to link frame.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool forwardKinematics(double *q, const Frame &xBase, int linkId, double *x) = 0;
        
        /** Compute the inverse dynamics.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity.
         * @param ddq Joint accelerations (rad/s^2).
         * @param ddxB Acceleration of the robot base in world reference frame, 3 values for linear and 3 for angular acceleration.
         * @param g gravity acceleration expressed in world frame (3 values)
         * @param tau Output generalized forces at the joints and base (N+6 dimensional, with N=number of joints).
         * @return True if the operation succeeded, false otherwise. */
        virtual bool inverseDynamics(double *q, const Frame &xBase, double *dq, double *dxB, double *ddq, double *ddxB, double *g, double *tau) = 0;
        
        /** Compute the floating base Mass Matrix.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame
         * @param M Output N+6xN+6 mass matrix, with N=number of joints.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool computeMassMatrix(double *q, const Frame &xBase, double *M) = 0;
        
        /** Compute the generalized bias forces (gravity+Coriolis+centrifugal) terms.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity.
         * @param g gravity acceleration expressed in world frame (3 values)
         * @param h Output N+6-dim vector containing all generalized bias forces (gravity+Coriolis+centrifugal), with N=number of joints.
         * @return True if the operation succeeded, false otherwise. */
        
        virtual bool computeGeneralizedBiasForces(double *q, const Frame &xBase, double *dq, double *dxB, double* g, double *h) = 0;
       
        /** Compute the 6 element centroidal momentum, as defined in:
         * Centroidal dynamics of a humanoid robot - DE Orin, A Goswami, SH Lee - Autonomous Robots 35 (2-3), 161-176
         * @param q Joint angles (in radians) 
         * @param xBase Rototranslation from world frame to robot base frame (\f${}^w H_b \f$)
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity
         * @param h output 6-element vector containg the centroidal momentum (3 value for linear momentum and 3 for angular momentum)
         * @return True if the operation succeeded, false otherwise. */
        virtual bool computeCentroidalMomentum(double *q, const Frame &xBase, double *dq, double *dxB, double *h) = 0;
        
    };
    
}

#endif //IWHOLEBODYMODEL_H
