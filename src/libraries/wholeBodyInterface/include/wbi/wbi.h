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

/**
 * \defgroup wbi wbi
 *
 * @ingroup codyco_libraries
 *
 * THIS CODE IS UNDER DEVELOPMENT!
 * Library defining a general interface for communicating with a floating-base rigid robot.
 * The interface is divided into four main parts:
 * - sensor: read sensor data (e.g. encoders, force/torque sensors, IMUs)
 * - state estimation: read estimations of the state of the robot (e.g. joint pos/vel/acc, external forces)
 * - actuator: send commands to the low-level motor controllers
 * - model: access the kinematic/dynamic model of the robot
 * The robot has n joints and n+6 DoFs, because of the 6 additional DoFs representing the position and 
 * orientation of its floating-base. 
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
 * All angles are expressed in radians.
 *
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \author Andrea Del Prete, Silvio Traversaro, Marco Randazzo - name.surname@iit.it
 *
 * Copyright (C) 2013-.. CODYCO
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 **/

#ifndef WBI_H
#define WBI_H

#include <wbi/wbiConstants.h>
#include <wbi/wbiUtil.h>
#include <vector>

namespace wbi
{

    /*
     * Interface for reading the sensors of the robot.
     */
    class iWholeBodySensors
    {
    public:
        /** Virtual destructor (to allow implementation of proper destructor in son classes). */
        inline virtual ~iWholeBodySensors(){}

        /** Initialize the object. This method should be called after adding the sensors,
         *  but before reading any sensor. */
        virtual bool init() = 0;
        /** Close all the communication channels with the robot. This method should be 
         *  called before destroying the object. */
        virtual bool close() = 0;

        /** Add the specified sensor so that it can be read. 
         * @param st Type of sensor.
         * @param sid Id of the sensor.
         * @return True if the sensor has been added, false otherwise (e.g. the sensor has been already added).
         */
        virtual bool addSensor(const SensorType st, const LocalId &sid) = 0;
        
        /** Add the specified sensors so that they can be read. 
         * @param st Type of sensors.
         * @param sids Ids of the sensors.
         * @return True if the sensor has been added, false otherwise (e.g. the sensor has been already added).
         */
        virtual int addSensors(const SensorType st, const LocalIdList &sids) = 0;

        /** Remove the specified sensor. 
         * @param st Type of the sensor to remove.
         * @param j Id of the sensor to remove.
         * @return True if the sensor has been removed, false otherwise.
         */
        virtual bool removeSensor(const SensorType st, const LocalId &sid) = 0;

        /** Remove all the sensors associated to the specified joint. This affects the reading of all the 
         *  joint space sensors (e.g. encoders, pwm).
         * @param j Id of the joint.
         * @return True if the operation succeeded, false otherwise.
         */
        //virtual bool removeSensorsOfJoint(const LocalId &j);
        
        /** Get a copy of the sensor list of the specified sensor type.
         * @param st Type of sensors.
         * @return A copy of the sensor list. */
        virtual const LocalIdList& getSensorList(const SensorType st) = 0;
        
        /** Get the number of sensors of the specified type.
         * @return The number of sensors of the specified type. */
        //virtual int getSensorNumber(const SensorType st) = 0;

        /** Read the specified sensor.
         * @param st Type of sensor to read.
         * @param sid Id of the sensor to read.
         * @param data Output data vector.
         * @param stamps Output vector of timestamps.
         * @param blocking If true, the reading is blocking, otherwise it is not.
         * @return True if all the readings succeeded, false otherwise.
         */
        virtual bool readSensor(const SensorType st, const LocalId &sid, double *data, double *stamps=0, bool blocking=true) = 0;
        
        /** Read all the sensors of the specified type.
         * @param st Type of the sensor to read.
         * @param sid Id of the sensor to read.
         * @param data Output data vector.
         * @param stamps Output vector of timestamps.
         * @param blocking If true, the reading is blocking, otherwise it is not.
         * @return True if the reading succeeded, false otherwise.
         */
        virtual bool readSensors(const SensorType st, double *data, double *stamps=0, bool blocking=true) = 0;
    };
    
    /**
      * Interface to access the estimates of the state of the robot.
      */
    class iWholeBodyStates
    {
    public:
        /** Virtual destructor (to allow implementation of proper destructor in son classes). */
        inline virtual ~iWholeBodyStates(){}
        virtual bool init() = 0;
        virtual bool close() = 0;

        /** Add the specified estimate so that it can be read. 
         * @param st Type of estimate.
         * @param sid Id of the estimate.
         * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
         */
        virtual bool addEstimate(const EstimateType st, const LocalId &sid) = 0;
        
        /** Add the specified estimates so that they can be read. 
         * @param st Type of estimates.
         * @param sids Ids of the estimates.
         * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
         */
        virtual int addEstimates(const EstimateType st, const LocalIdList &sids) = 0;

        /** Remove the specified estimate. 
         * @param st Type of the estimate to remove.
         * @param j Id of the estimate to remove.
         * @return True if the estimate has been removed, false otherwise.
         */
        virtual bool removeEstimate(const EstimateType st, const LocalId &sid) = 0;

        /** Remove all the estimates associated to the specified joint.
         * @param j Id of the joint.
         * @return True if the operation succeeded, false otherwise.
         */
        //virtual bool removeEstimatesOfJoint(const LocalId &j);
        
        /** Get a copy of the estimate list of the specified estimate type.
         * @param st Type of estimate.
         * @return A copy of the estimate list. */
        virtual const LocalIdList& getEstimateList(const EstimateType st) = 0;
        
        /** Get the number of estimates of the specified type.
         * @return The number of estimates of the specified type. */
        //virtual int getEstimateNumber(const EstimateType st) = 0;

        /** Get the estimate of the specified quantity at the specified time.
         * @param et Type of estimate to get.
         * @param sid Id of the estimate
         * @param data Output data vector.
         * @param time Time at which to estimate the quantity.
         * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
         * @return True if all the estimate succeeded, false otherwise.
         */
        virtual bool getEstimate(const EstimateType et, const LocalId &sid, double *data, double time=-1.0, bool blocking=true) = 0;

        /** Get all the estimates of the specified estimate type at the specified time.
         * @param et Type of estimate to get.
         * @param data Output data vector.
         * @param time Time at which to estimate the quantity.
         * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
         * @return True if all the estimate succeeded, false otherwise.
         */
        virtual bool getEstimates(const EstimateType et, double *data, double time=-1.0, bool blocking=true) = 0;

        /** Set the value of the specified parameter of the estimation algorithm
         * of the specified estimate type.
         * @param et Estimation type (e.g. joint velocity, motor torque).
         * @param ep Parameter to set.
         * @param value Value of the parameter to set.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool setEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value) = 0;
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

        inline virtual ~iWholeBodyModel(){}
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
          * @param tau Output generalized forces at the joints and base (N+6 dimensional, with N=number of joints).
         * @return True if the operation succeeded, false otherwise. */
        virtual bool inverseDynamics(double *q, const Frame &xBase, double *dq, double *dxB, double *ddq, double *ddxB, double *tau) = 0;

        /** Compute the direct dynamics.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity.
         * @param M Output N+6xN+6 mass matrix, with N=number of joints.
         * @param h Output N+6-dim vector containing all generalized bias forces (gravity+Coriolis+centrifugal), with N=number of joints.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool directDynamics(double *q, const Frame &xBase, double *dq, double *dxB, double *M, double *h) = 0;
    };
    
    /**
      * Interface to the actuators of the robot.
      */
    class iWholeBodyActuators
    {
    public:
        inline virtual ~iWholeBodyActuators(){}
        virtual bool init() = 0;
        virtual bool close() = 0;

        //virtual int getActuatorNumber() = 0;
        virtual bool removeActuator(const LocalId &j) = 0;
        virtual bool addActuator(const LocalId &j) = 0;
        virtual int addActuators(const LocalIdList &j) = 0;
        virtual const LocalIdList& getActuatorList() = 0;
        
        /** Set the control mode of the specified joint(s).
          * @param controlMode Id of the control mode.
          * @param ref Reference value(s) for the controller.
          * @param joint Joint number, if negative, all joints are considered.
          * @return True if operation succeeded, false otherwise. */
        virtual bool setControlMode(ControlMode controlMode, double *ref=0, int joint=-1) = 0;
        
        /** Set the reference value for the controller of the specified joint(s).
          * @param ref Reference value(s) for the controller.
          * @param joint Joint number, if negative, all joints are considered.
          * @return True if operation succeeded, false otherwise. */
        virtual bool setControlReference(double *ref, int joint=-1) = 0;

        /** Set a parameter (e.g. a gain) of one or more joint controllers.
          * @param paramId Id of the parameter.
          * @param value Value(s) of the parameter.
          * @param joint Joint number, if negative, all joints are considered.
          * @return True if operation succeeded, false otherwise. */
        virtual bool setControlParam(ControlParam paramId, const void *value, int joint=-1) = 0;
    };
    
    /**
      * Interface to state estimations, kinematic/dynamic model and actuators of the robot.
      */
    class wholeBodyInterface: public iWholeBodyStates, public iWholeBodyModel, public iWholeBodyActuators
    {
    public:
        inline virtual ~wholeBodyInterface(){}
        virtual bool init() = 0;
        virtual bool close() = 0;
        
        /** Remove the actuator, model and all the estimates associated to the specified joint.
          * @param j Id of the joint.
          * @return True if the operation succeeded, false otherwise. */
        virtual bool removeJoint(const LocalId &j) = 0;
        
        /** Add the actuator, model and all the estimates associated to the specified joint.
          * @param j Id of the joint.
          * @return True if the operation succeeded, false otherwise. */
        virtual bool addJoint(const LocalId &j) = 0;

        /** Add the actuators, models and all the estimates associated to the specified joints.
          * @param j Id of the joint.
          * @return True if the operation succeeded, false otherwise. */
        virtual int addJoints(const LocalIdList &j) = 0;
    };
    
    
} // end namespace

#endif

