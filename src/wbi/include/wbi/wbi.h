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
 * Authors: Serena Ivaldi, Andrea Del Prete, Marco Randazzo
 * email: serena.ivaldi@isir.upmc.fr - andrea.delprete@iit.it - marco.randazzo@iit.it
 */

/**
 * \defgroup wbi wbi
 *
 * @ingroup codyco_libraries
 *
 * THIS CODE IS UNDER DEVELOPMENT!
 * Library defining a general interface for communicating with a robot.
 *
 * \section dep_sec Dependencies
 * None.
 *
 * \section intro_sec Description
 * We assume the robot is divided into subparts, which we call "body parts" (e.g. left arm, right leg, torso).
 * Each body part has an unique integer identifier.
 * In each body part there exists a unique local identifier associated to any object (e.g. joint, sensor, motor) 
 * that belongs to that body part.
 * Each object also has a unique global identifier, which defines how the objects are serialized at whole-body level.
 *
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \author Andrea Del Prete - andrea.delprete@iit.it
 *
 * Copyright (C) 2013-.. CODYCO
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 **/

#ifndef WBI_H
#define WBI_H

#include <wbi/wbiUtil.h>
#include <vector>

namespace wbi
{
    
    enum ControlMode { CTRL_MODE_OPEN_LOOP, CTRL_MODE_POS, CTRL_MODE_TORQUE, CTRL_MODE_VEL};
    
    /*
     * Interface for reading the sensors of the robot.
     */
    class iWholeBodySensors
    {
    public:
        /** Initialize the object. This method should be called after adding the joints,
         *  but before reading any sensor. */
        virtual bool init() = 0;
        /** Close all the communication channels with the robot. This method should be 
         *  called before destroing the object. */
        virtual bool close() = 0;
        
        /** Remove the specified joint from the list. This affects the reading of all the 
         *  joint space sensors (e.g. encoders, pwm).
         * @param j Id of the joint to remove.
         * @return True if the joint has been removed, false otherwise.
         */
        virtual bool removeJoint(const LocalId &j) = 0;
        /** Add the specified joint to the list. This affects the reading of all the 
         *  joint space sensors (e.g. encoders, pwm).
         * @param j Id of the joint to add.
         * @return True if the joint has been added, false otherwise.
         */
        virtual bool addJoint(const LocalId &j) = 0;
        /** Add the specified joints to the list. This affects the reading of all the 
         *  joint space sensors (e.g. encoders, pwm).
         * @param j Ids of the joints to add.
         * @return The number of joints added to the list.
         */
        virtual int addJoints(const LocalIdList &j) = 0;
        /** Get a copy of the joint list.
         * @return A copy of the joint list. */
        virtual LocalIdList getJointList() = 0;
        /** Get the number of joints in the joint list.
         * @return The number of degrees of freedom. */
        virtual int getDoFs() = 0;

        virtual bool addIMU(const LocalId &i) = 0;
        virtual bool removeIMU(const LocalId &i) = 0;
        virtual LocalIdList getIMUList() = 0;
        virtual bool addFTsensor(const LocalId &i) = 0;
        virtual bool removeFTsensor(const LocalId &i) = 0;
        virtual LocalIdList getFTsensorList() = 0;
        
        /** Read the joint encoders.
         * @param q Output vector of joint angles
         * @param stamps Output vector of timestamps
         * @param wait If true, the reading is blocking, otherwise it is not
         * @return True if all the readings succeeded, false otherwise.
         */
        virtual bool readEncoders(double *q, double *stamps=0, bool wait=true) = 0;
        /** Read the motor PWMs (proportional to the motor voltages).
         * @param pwm Output vector of motor PWMs
         * @param stamps Output vector of timestamps
         * @param wait If true, the reading is blocking, otherwise it is not
         * @return True if all the readings succeeded, false otherwise.
         */
        virtual bool readPwm(double *pwm, double *stamps=0, bool wait=true) = 0;
        /** Read the inertial measurement unit (angular vel: w, angular acc: dw, linear acc: ddp).
         * @param inertial Output vector of inertial measurements (w, dw, ddp)
         * @param stamps Output vector of timestamps
         * @param wait If true, the reading is blocking, otherwise it is not
         * @return True if all the readings succeeded, false otherwise.
         */
        virtual bool readIMU(double *inertial, double *stamps=0, bool wait=true) = 0;
        /** Read the force/torque sensors (3D force + 3D moment).
         * @param ft Output vector of force/torque measurements (f, m)
         * @param stamps Output vector of timestamps
         * @param wait If true, the reading is blocking, otherwise it is not
         * @return True if all the readings succeeded, false otherwise.
         */
        virtual bool readFTsensors(double *ft, double *stamps=0, bool wait=true) = 0;
    };
    
    /**
      * Interface to access the estimates of the state of the robot.
      */
    class iWholeBodyStates
    {
    public:
        virtual bool init() = 0;
        virtual bool close() = 0;

        virtual int getDoFs() = 0;
        virtual bool removeJoint(const LocalId &j) = 0;
        virtual bool addJoint(const LocalId &j) = 0;
        virtual int addJoints(const LocalIdList &j) = 0;
        virtual LocalIdList getJointList() = 0;
        
        virtual bool addIMU(const LocalId &i) = 0;
        virtual bool removeIMU(const LocalId &i) = 0;
        virtual LocalIdList getIMUList() = 0;
        
        virtual bool addFTsensor(const LocalId &i) = 0;
        virtual bool removeFTsensor(const LocalId &i) = 0;
        virtual LocalIdList getFTsensorList() = 0;
        
        virtual bool getQ(double *q, double time=-1.0, bool wait=false) = 0;
        virtual bool getDq(double *dq, double time=-1.0, bool wait=false) = 0;
        virtual bool getDqMotors(double *dqM, double time=-1.0, bool wait=false) = 0;
        virtual bool getD2q(double *d2q, double time=-1.0, bool wait=false) = 0;
        virtual bool getPwm(double *pwm, double time=-1.0, bool wait=false) = 0;
        virtual bool getInertial(double *inertial, double time=-1.0, bool wait=false) = 0;
        virtual bool getFTsensors(double *ftSens, double time=-1.0, bool wait=false) = 0;
        virtual bool getTorques(double *tau, double time=-1.0, bool wait=false) = 0;
        //virtual bool getExternalForces(double *fExt, double time=-1.0, bool wait=false) = 0;
    };
    
    
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

        virtual bool init() = 0;
        virtual bool close() = 0;

        virtual int getDoFs() = 0;

        /** Remove the specified joint form the robot model. The joint is considered blocked
          * at its last known value (values of the joint angles are stored whenever a method of 
          * iWholeBodyModel is called). If no previous value of the joint angle is known, zero is assumed.
          * @param j Id of the joint to remove
          * @return True if the joint was found and removed, false otherwise. */
        virtual bool removeJoint(const LocalId &j) = 0;
        virtual bool addJoint(const LocalId &j) = 0;
        virtual int addJoints(const LocalIdList &j) = 0;
        virtual LocalIdList getJointList() = 0;
        
        virtual bool getJointLimits(double *qMin, double *qMax, int joint=-1) = 0;

        /** Get the id of the link with the specified name.
          * @param linkName Name of the link.
          * @param linkId Id of the link (if found).
          * @return True if the link name was found, false otherwise. */
        virtual bool getLinkId(const char *linkName, int &linkId) = 0;
        
        /** Compute rototranslation matrix from root reference frame to reference frame associated to the specified link.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link that is the target of the rototranslation
          * @param H Output 4x4 rototranslation matrix (stored by rows)
          * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeH(double *q, const Frame &xBase, int linkId, double *H) = 0;
        
        /** Compute the Jacobian of the specified point of the robot.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link
          * @param J Output 6xN Jacobian matrix (stored by rows), where N=number of joints
          * @param pos 3d position of the point expressed w.r.t the link reference frame
          * @return True if the operation succeeded, false otherwise (invalid input parameters) 
          * @note If linkId==COM_LINK_ID then the angular part of J is related to the angular velocity of the
          *       reference frame associated to the CoM. This Jacobian premultiplied by the whole robot's inertia
          *       matrix is equal to the Jacobian of the angular momentum of the whole robot. */
        virtual bool computeJacobian(double *q, const Frame &xBase, int linkId, double *J, double *pos=0) = 0;
        
        /** Given a point on the robot, compute the product between the time derivative of its 
          * Jacobian and the joint velocity vector.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param dq Joint velocities
          * @param linkId Id of the link
          * @param dJdq Output 6-dim vector containing the product dJ*dq 
          * @param pos 3d position of the point expressed w.r.t the link reference frame
          * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeDJdq(double *q, const Frame &xBase, double *dq, double *dxB, int linkId, double *dJdq, double *pos=0) = 0;
        
        /** Compute the forward kinematics of the specified joint.
          * @param q Joint angles.
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link.
          * @param x Output 7-dim pose vector (3 for pos, 4 for quaternion orientation).
          * @return True if operation succeeded, false otherwise. */
        virtual bool forwardKinematics(double *q, const Frame &xBase, int linkId, double *x) = 0;
        
        /** Compute the inverse dynamics.
          * @param q Joint angles.
          * @param xBase Rototranslation from world frame to robot base frame
          * @param dq Joint velocities.
          * @param dxB Velocity of the robot base, 3 values for linear velocity and 3 values for angular velocity.
          * @param ddq Joint accelerations.
          * @param ddxB Acceleration of the robot base, 3 values for linear acceleration and 3 values for angular acceleration.
          * @param tau Output joint torques.
         * @return True if operation succeeded, false otherwise. */
        virtual bool inverseDynamics(double *q, const Frame &xBase, double *dq, double *dxB, double *ddq, double *ddxB, double *tau) = 0;

        /** Compute the direct dynamics.
         * @param q Joint angles.
         * @param xBase Rototranslation from world frame to robot base frame
         * @param dq Joint velocities.
         * @param dxB Velocity of the robot base, 3 values for linear velocity and 3 values for angular velocity.
         * @param M Output N+6xN+6 mass matrix, with N=number of joints.
         * @param h Output N+6-dim vector containing all generalized bias forces (gravity+Coriolis+centrifugal).
         * @return True if operation succeeded, false otherwise. */
        virtual bool directDynamics(double *q, const Frame &xBase, double *dq, double *dxB, double *M, double *h) = 0;
    };
    
    /**
      * Interface to the actuators of the robot.
      */
    class iWholeBodyActuators
    {
    public:
        virtual bool init() = 0;
        virtual bool close() = 0;

        virtual int getDoFs() = 0;
        virtual bool removeJoint(const LocalId &j) = 0;
        virtual bool addJoint(const LocalId &j) = 0;
        virtual int addJoints(const LocalIdList &j) = 0;
        virtual LocalIdList getJointList() = 0;
        
        /** Set the control mode of the specified joint(s).
          * @param controlMode Id of the control mode
          * @param joint Joint number, if negative, all joints are considered
          * @return True if operation succeeded, false otherwise */
        virtual bool setControlMode(int controlMode, int joint=-1) = 0;
        virtual bool setTorqueRef(double *taud, int joint=-1) = 0;
        virtual bool setPosRef(double *qd, int joint=-1) = 0;
        virtual bool setVelRef(double *dqd, int joint=-1) = 0;
        virtual bool setPwmRef(double *pwmd, int joint=-1) = 0;
        virtual bool setReferenceSpeed(double *rspd, int joint=-1) = 0;
    };
    
    /**
      * Interface to state estimations, kinematic/dynamic model and actuators of the robot.
      */
    class wholeBodyInterface: public iWholeBodyStates, public iWholeBodyModel, public iWholeBodyActuators
    {
    public:
        virtual bool init() = 0;
        virtual bool close() = 0;

        virtual int getDoFs() = 0;
        virtual bool removeJoint(const LocalId &j) = 0;
        virtual bool addJoint(const LocalId &j) = 0;
        virtual int addJoints(const LocalIdList &j) = 0;
        virtual LocalIdList getJointList() = 0;
    };
    
    
} // end namespace

#endif

