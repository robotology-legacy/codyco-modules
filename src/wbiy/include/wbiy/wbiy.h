
/*
 * Copyright (C) 2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email: marco.randazzo@iit.it
 *
 * Further modifications
 *
 * Copyright (C) 2013 CoDyCo Consortium
 * Author: Serena Ivaldi
 * email: serena.ivaldi@isir.upmc.fr
 *
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

#ifndef WBIY_H
#define WBIY_H

#include <yarp/dev/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <wbi/wbi.h>
#include <map>
#if __APPLE__
#include <tr1/unordered_map>
#else
#include <unordered_map>
#endif
#include <vector>

/* CODE UNDER DEVELOPMENT */

namespace wbiy
{
    
    struct HashLocalId
    {
        std::size_t operator()(const wbi::LocalId &b) const
        { return b.bodyPart*100 + b.index; }
    };
    
#if __APPLE__
    typedef std::tr1::unordered_map<wbi::LocalId, unsigned int, HashLocalId> jointMap;
#else 
    typedef std::unordered_map<wbi::LocalId, unsigned int, HashLocalId> jointMap;
#endif
    
    bool openPolyDriver(const std::string &localName, const std::string &robotName, yarp::dev::PolyDriver *&pd, const std::string &bodyPartName);
    
//    bool updateLocal2GlobalIndex(wbi::LocalIdList &jId, unsigned int dof, jointMap &l2g, std::vector<wbi::LocalId> &g2l);
    
    
    /*
     * Class for reading the sensors of a robot that is accessed through
     * a yarp interface (i.e. PolyDrivers)
     */
    class yarpWholeBodySensors: public wbi::iWholeBodySensors
    {
    protected:
        bool                        initDone;
        int                         dof;            // number of degrees of freedom
        std::string                 name;           // name used as root for the local ports
        std::string                 robot;          // name of the robot
        std::vector<int>            bodyParts;      // list of the body parts
        std::vector<std::string>    bodyPartNames;  // names of the body parts
        std::map<int,unsigned int>  bodyPartAxes;   // number of axes for each body part
        wbi::LocalIdList            jointIdList;    // list of the joint ids
        
        // last reading data
        std::map<int, yarp::sig::Vector>  qLastRead;
        std::map<int, yarp::sig::Vector>  qStampLastRead;
        std::map<int, yarp::sig::Vector>  pwmLastRead;

        // yarp interfaces
        std::map<int, yarp::dev::IEncoders*>            ienc;
        std::map<int, yarp::dev::IOpenLoopControl*>     iopl;
        std::map<int, yarp::dev::PolyDriver*>           dd;
        
        bool openDrivers(int bodyPart);
        void setBodyPartName(int bodyPart, const std::string &nameBodyPart);
        
    public:
        // *** CONSTRUCTORS ***
        /**
          * @param _name Local name of the interface (used as stem of port names)
          * @param _robotName Name of the robot
          * @param _bodyPartNames Array of names of the body part (used when opening the polydrivers)
          */
        yarpWholeBodySensors(const char* _name, const char* _robotName, const std::vector<std::string> &_bodyPartNames);

        virtual bool init();
        virtual bool removeJoint(const wbi::LocalId &j);
        virtual bool addJoint(const wbi::LocalId &j);
        virtual int addJoints(const wbi::LocalIdList &j);
        virtual int getDoFs(){ return dof; }
        
        virtual bool readEncoders(double *q, double *stamps, bool wait=true);
        virtual bool readPwm(double *pwm, double *stamps, bool wait=true);
        virtual bool readInertial(double *inertial, double *stamps, bool wait=true);
        virtual bool readFTsensors(double *ftSens, double *stamps, bool wait=true);
    };
    
    
    /*
     * Class for interfacing the actuators of a robot that is accessed through
     * a yarp interface (polydrivers etc)
     */
    class yarpWholeBodyActuators : public wbi::iWholeBodyActuators
    {
    protected:
        bool                        initDone;
        int                         dof;            // number of degrees of freedom considered
        std::string                 name;           // name used as root for the local ports
        std::string                 robot;          // name of the robot
        std::vector<int>            bodyParts;      // list of the body parts
        std::vector<std::string>    bodyPartNames;  // names of the body parts
        wbi::LocalIdList            jointIdList;    // list of the joint ids
        
        // yarp drivers
        std::map<int, yarp::dev::IPositionControl*>     ipos;
        std::map<int, yarp::dev::ITorqueControl*>       itrq;
        std::map<int, yarp::dev::IImpedanceControl*>    iimp;
        std::map<int, yarp::dev::IControlMode*>         icmd;
        std::map<int, yarp::dev::IVelocityControl*>     ivel;
        std::map<int, yarp::dev::IOpenLoopControl*>     iopl;
        std::map<int, yarp::dev::PolyDriver*>           dd;
        
        bool openDrivers(int bodyPart);
        
    public:
        // *** CONSTRUCTORS ***
        yarpWholeBodyActuators(const char* _name, const char* _robotName, const std::vector<std::string> &_bodyPartNames);
        
        virtual bool init();
        virtual int getDoFs(){ return dof; }
        virtual bool removeJoint(const wbi::LocalId &j);
        virtual bool addJoint(const wbi::LocalId &j);
        virtual int addJoints(const wbi::LocalIdList &j);
        
        virtual bool setControlMode(int controlMode, int joint=-1);
        virtual bool setTorqueRef(double *taud, int joint=-1);
        virtual bool setPosRef(double *qd, int joint=-1);
        virtual bool setVelRef(double *dqd, int joint=-1);
        virtual bool setPwmRef(double *pwmd, int joint=-1);
    };
    
    
    
    /**
     * Class to access the estimates of the states of a robot with a yarp interface.
     */
    class robotWholeBodyStates : public wbi::iWholeBodyStates
    {
    protected:
        bool                        initDone;
        int                         dof;            // number of degrees of freedom considered
        std::string                 name;           // name used as root for the local ports
        std::string                 robot;          // name of the robot
        std::vector<int>            bodyParts;      // list of the body parts
        std::map<int,std::string>   bodyPartsName;  // name of the body parts
        wbi::LocalIdList            jointIdList;    // list of the joint ids
        
        yarpWholeBodySensors        *sensors;       // interface to access the robot sensors
        double                      estWind;        // time window for the estimation
        
    public:
        // *** CONSTRUCTORS ***
        robotWholeBodyStates(const char* _name, const char* _robotName, double estimationTimeWindow);
        
        virtual bool init();
        virtual int getDoFs();
        virtual bool removeJoint(const wbi::LocalId &j);
        virtual bool addJoint(const wbi::LocalId &j);
        virtual int addJoints(const wbi::LocalIdList &j);
        
        virtual bool getQ(double *q, double time=-1.0, bool wait=false);
        virtual bool getDq(double *dq, double time=-1.0, bool wait=false);
        virtual bool getDqMotors(double *dqM, double time=-1.0, bool wait=false);
        virtual bool getD2q(double *d2q, double time=-1.0, bool wait=false);
        virtual bool getPwm(double *pwm, double time=-1.0, bool wait=false);
        virtual bool getInertial(double *inertial, double time=-1.0, bool wait=false);
        virtual bool getFTsensors(double *ftSens, double time=-1.0, bool wait=false);
        virtual bool getTorques(double *tau, double time=-1.0, bool wait=false);
    };
    
    
    /**
     * Interface to the kinematic/dynamic model of the robot.
     */
    class icubWholeBodyModel: public wbi::iWholeBodyModel
    {
    public:
        virtual bool init();
        virtual int getDoFs();
        virtual bool removeJoint(const wbi::LocalId &j);
        virtual bool addJoint(const wbi::LocalId &j);
        virtual int addJoints(const wbi::LocalIdList &j);
        virtual bool getJointLimits(double *qMin, double *qMax, int joint=-1);
        
        /** Compute rototranslation matrix from root reference frame to reference frame associated to the specified link.
         * @param q Joint angles
         * @param xBase Pose of the robot base, 3 values for position and 4 values for orientation
         * @param linkId Id of the link that is the target of the rototranslation
         * @param H Output 4x4 rototranslation matrix (stored by rows)
         * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeH(double *q, double *xBase, int linkId, double *H);
        
        /** Compute the Jacobian of the specified point of the robot.
         * @param q Joint angles
         * @param xBase Pose of the robot base, 3 values for position and 4 values for orientation
         * @param linkId Id of the link
         * @param J Output 6xN Jacobian matrix (stored by rows), where N=number of joints
         * @param pos 3d position of the point expressed w.r.t the link reference frame
         * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeJacobian(double *q, double *xBase, int linkId, double *J, double *pos=0);
        
        /** Given a point on the robot, compute the product between the time derivative of its 
         * Jacobian and the joint velocity vector.
         * @param q Joint angles
         * @param xBase Pose of the robot base, 3 values for position and 4 values for orientation
         * @param dq Joint velocities
         * @param linkId Id of the link
         * @param dJdq Output 6-dim vector containing the product dJ*dq 
         * @param pos 3d position of the point expressed w.r.t the link reference frame
         * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeDJdq(double *q, double *xB, double *dq, double *dxB, int linkId, double *dJdq, double *pos=0);
        
        /** Compute the forward kinematics of the specified joint.
         * @param q Joint angles
         * @param xB Pose of the robot base, 3 values for position and 4 values for orientation
         * @param linkId Id of the link
         * @param x Output 7-dim pose vector (3 for pos, 4 for angle-axis orientation)
         * @return True if operation succeeded, false otherwise */
        virtual bool forwardKinematics(double *q, double *xB, int linkId, double *x);
        
        /** Compute the inverse dynamics.
         * @param q Joint angles
         * @param xB Pose of the robot base, 3 values for position and 4 values for orientation
         * @param dq Joint velocities
         * @param dxB Velocity of the robot base, 3 values for linear velocity and 3 values for angular velocity
         * @param ddq Joint accelerations
         * @param ddxB Acceleration of the robot base, 3 values for linear acceleration and 3 values for angular acceleration
         * @param tau Output joint torques
         * @return True if operation succeeded, false otherwise */
        virtual bool inverseDynamics(double *q, double *xB, double *dq, double *dxB, double *ddq, double *ddxB, double *tau);
        
        /** Compute the direct dynamics.
         * @param q Joint angles
         * @param xB Pose of the robot base, 3 values for position and 4 values for orientation
         * @param dq Joint velocities
         * @param dxB Velocity of the robot base, 3 values for linear velocity and 3 values for angular velocity
         * @param M Output NxN mass matrix, with N=number of joints
         * @param h Output N-dim vector containing all generalized bias forces (gravity+Coriolis+centrifugal) 
         * @return True if operation succeeded, false otherwise */
        virtual bool directDynamics(double *q, double *xB, double *dq, double *dxB, double *M, double *h);
    };
    
    
    /**
     * Class to communicate with iCub
     */
    class icubWholeBodyInterface : 
        public wbi::wholeBodyInterface, public robotWholeBodyStates, public yarpWholeBodyActuators, public icubWholeBodyModel
    {
    public:
        // *** CONSTRUCTORS ***
        icubWholeBodyInterface(const char* _name, const char* _robotName, double estimationTimeWindow);
        
        virtual bool init();
        virtual int getDoFs();
        virtual bool removeJoint(const wbi::LocalId &j);
        virtual bool addJoint(const wbi::LocalId &j);
        virtual int addJoints(const wbi::LocalIdList &j);
    };
    
    
    
} // end namespace

#endif

