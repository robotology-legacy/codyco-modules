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

#ifndef WBI_H
#define WBI_H

#include <vector>
#include <map>
#include <string>


/*
 * THIS CODE IS UNDER DEVELOPMENT!
 */


namespace wbi
{
    
    
    enum ControlMode { CTRL_MODE_OPEN_LOOP, CTRL_MODE_POS, CTRL_MODE_TORQUE, CTRL_MODE_VEL};
    
    // iterate over all body parts of the specified jointIds
#define FOR_ALL_BODY_PARTS_OF(itBp, jIds)   for (JointIds::iterator itBp=jIds.begin(); itBp!=jIds.end(); itBp++)
    // iterate over all joints of the specified body part
#define FOR_ALL_JOINTS(itBp, itJ)           for(vector<unsigned int>::const_iterator itJ=itBp->second.begin(); itJ!=itBp->second.end(); itJ++)
    // as before, does not uses a const_iterator -> to be used for deleting and similar operations
    #define FOR_ALL_JOINTS_NC(itBp, itJ)    for(vector<unsigned int>::iterator itJ=itBp->second.begin(); itJ!=itBp->second.end(); itJ++)
    // iterate over all joints of all body parts of the specified jointIds
#define FOR_ALL_OF(itBp, itJ, jIds)         FOR_ALL_BODY_PARTS_OF(itBp, jIds) FOR_ALL_JOINTS(itBp, itJ)
    
    /**
     * Joint identifier, composed by the body part identifier and the joint number.
     */
    class JointId
    {
    public:
        /// body part
        int bp;
        /// joint
        unsigned int joint;
        /// joint description
        std::string joint_description;
        
        JointId(): bp(0), joint(0) {}
        JointId(int _bp, unsigned int _j): bp(_bp), joint(_j) {}
        JointId(int _bp, unsigned int _j, std::string _desc): bp(_bp), joint(_j), joint_description(_desc) {}
        bool operator==(const JointId &other) const { return (bp==other.bp && joint==other.joint); }
    };
    
    /**
     * List of joint identifiers, that is a map from body part identifiers to lists of joint numbers.
     */
    class JointIds : public std::map< int, std::vector<unsigned int> >
    {
    public:
        /** Remove the specified joint from the list */
        virtual bool removeJoint(const JointId &j);
        
        /** Add the specified joint to the list.
         * @param j Joint to add
         * @return true if the joint has been added, false if it was already present
         */
        virtual bool addJoint(const JointId &j);
        
        /** Add the specified joints to the list.
         * @param j Joints to add
         * @return true if all joints have been added, false it at least one of them was already present
         */
        virtual bool addJoints(const JointIds &j);
        
        /** Get the number of degrees of freedom */
        virtual unsigned int getDoFs();
        
        /* Check whether the specified body part is present in this list. */
        virtual bool containsBodyPart(int bp){ return !(find(bp)==end()); }
        
    };
    
    
    /**
     * Sensor identifier, composed by the body part identifier and the sensor number.
     */
    class SensorId
    {
    public:
        /// body part
        int bp;
        /// sensor
        unsigned int sensor;
        /// sensor description
        std::string sensor_description;
        
        SensorId(): bp(0), sensor(0) {}
        SensorId(int _bp, unsigned int _s): bp(_bp), sensor(_s) {}
        SensorId(int _bp, unsigned int _s, std::string _desc): bp(_bp), sensor(_s), sensor_description(_desc) {}
        bool operator==(const SensorId &other) const { return (bp==other.bp && sensor==other.sensor); }
    };
    
    /**
     * List of sensor identifiers, that is a map from body part identifiers to lists of sensor numbers.
     */
    class SensorIds : public std::map< int, std::vector<unsigned int> >
    {
    public:
        /** Remove the specified sensor from the list */
        virtual bool removeSensor(const SensorId &j);
        
        /** Add the specified sensor to the list.
         * @param s Sensor to add
         * @return true if the sensor has been added, false if it was already present
         */
        virtual bool addSensor(const SensorId &s);
        
        /** Add the specified sensor to the list.
         * @param s Sensors to add
         * @return true if all sensors have been added, false it at least one of them was already present
         */
        virtual bool addSensors(const SensorIds &s);
        
        /* Check whether the specified body part is present in this list. */
        virtual bool containsBodyPart(int bp){ return !(find(bp)==end()); }
        
    };
    
    
    
    /*
     * Interface for all the sensors of a system
     */
    class iWholeBodySensors
    {
        
    public:
        virtual bool init() = 0;
        
        virtual bool removeJoint(const JointId &j) = 0;
        virtual bool addJoint(const JointId &j) = 0;
        virtual bool addJoints(const JointIds &j) = 0;
        
        virtual unsigned int getDoFs() = 0;
        
        virtual bool readEncoders(double *q, double *stamps, bool wait=true) = 0;
        virtual bool readPwm(double *pwm, double *stamps, bool wait=true) = 0;
        virtual bool readInertial(double *inertial, double *stamps, bool wait=true) = 0;
        virtual bool readFTsensors(double *ftSens, double *stamps, bool wait=true) = 0;
    };
    
    
    class iWholeBodyStates
    {
    public:
        virtual bool init() = 0;
        virtual unsigned int getDoFs() = 0;
        virtual bool removeJoint(const JointId &j) = 0;
        virtual bool addJoint(const JointId &j) = 0;
        virtual bool addJoints(const JointIds &j) = 0;
        
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
    
    
    class iWholeBodyModel
    {
    public:
        virtual bool init() = 0;
        virtual unsigned int getDoFs() = 0;
        virtual bool removeJoint(const JointId &j) = 0;
        virtual bool addJoint(const JointId &j) = 0;
        virtual bool addJoints(const JointIds &j) = 0;
        
        virtual bool computeH(double *q, unsigned int joint, double *H) = 0;
        virtual bool computeJacobian(double *q, unsigned int joint, double *J, double *pos=0) = 0;
        virtual bool computeDJdq(double *q, double *dq, unsigned int joint, double *dJdq, double *pos=0) = 0;
        virtual bool forwardKinematics(double *q, unsigned int joint, double *x) = 0;
        virtual bool inverseDynamics(double *q, double *dq, double *ddq, double *tau, double *wb=0, double *dwb=0, double *ddxb=0) = 0;
        virtual bool directDynamics(double *q, double *dq, double *M, double *h) = 0;
    };
    
    
    class iWholeBodyActuators
    {
    public:
        virtual bool init() = 0;
        virtual unsigned int getDoFs() = 0;
        virtual bool removeJoint(const JointId &j) = 0;
        virtual bool addJoint(const JointId &j) = 0;
        virtual bool addJoints(const JointIds &j) = 0;
        
        virtual bool setControlMode(int controlMode, int joint=-1) = 0;
        virtual bool setTorqueRef(double *taud, int joint=-1) = 0;
        virtual bool setPosRef(double *qd, int joint=-1) = 0;
        virtual bool setVelRef(double *dqd, int joint=-1) = 0;
        virtual bool setPwmRef(double *pwmd, int joint=-1) = 0;
    };
    
    
    class wholeBodyInterface: public iWholeBodyStates, public iWholeBodyModel, public iWholeBodyActuators
    {
    public:
        virtual bool init() = 0;
        virtual unsigned int getDoFs() = 0;
        virtual bool removeJoint(const JointId &j) = 0;
        virtual bool addJoint(const JointId &j) = 0;
        virtual bool addJoints(const JointIds &j) = 0;
    };
    
    
} // end namespace

#endif

