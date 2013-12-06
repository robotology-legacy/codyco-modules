/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete
 * email: andrea.delprete@iit.it
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

#ifndef WBI_ICUB_H
#define WBI_ICUB_H

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <iCub/iDynTree/iCubTree.h>
#include <wbiIcub/wbiIcubUtil.h>
#include <map>
//#if __APPLE__
//#include <tr1/unordered_map>
//#else
//#include <unordered_map>
//#endif

/* CODE UNDER DEVELOPMENT */

namespace wbiIcub
{
    /*
     * Class for reading the sensors of iCub.
     */
    class icubWholeBodySensors: public wbi::iWholeBodySensors
    {
    protected:
        bool                        initDone;
        std::string                 name;           // name used as root for the local ports
        std::string                 robot;          // name of the robot

        std::vector<int>            bodyParts;      // list of the body parts
        std::vector<std::string>    bodyPartNames;  // names of the body parts
        std::vector<id_2_PortName>  ftSens_2_port;  // list containing the port name for each force/torque sensor
        std::vector<id_2_PortName>  imu_2_port;     // list containing the port name for each IMU
        std::map<int,unsigned int>  bodyPartAxes;   // number of axes for each body part
        
        wbi::LocalIdList            emptyList;      ///< empty list of IDs to return in case of error
        wbi::LocalIdList            encoderIdList;  // list of the joint encoder ids
        wbi::LocalIdList            pwmSensIdList;  // list of the motor PWM sensor ids
        wbi::LocalIdList            imuIdList;      // list of the IMU ids
        wbi::LocalIdList            ftSensIdList;   // list of the force/torque sensor ids
        wbi::LocalIdList            torqueSensorIdList; //list of the torque sensor ids
        
        // LAST READING DATA (map body parts to data)
        std::map<int, yarp::sig::Vector>            qLastRead;
        std::map<int, yarp::sig::Vector>            qStampLastRead;
        std::map<int, yarp::sig::Vector>            pwmLastRead;
        std::map<int, yarp::sig::Vector>            torqueSensorsLastRead;
        // the key of these maps is the sensor id
        std::map<wbi::LocalId, yarp::sig::Vector>  imuLastRead;
        std::map<wbi::LocalId, yarp::sig::Vector>  ftSensLastRead;

        // yarp interfaces (the key of the maps is the body part)
        std::map<int, yarp::dev::IEncodersTimed*>       ienc;   // interface to read encoders
        std::map<int, yarp::dev::IOpenLoopControl*>     iopl;   // interface to read motor PWM
        std::map<int, yarp::dev::PolyDriver*>           dd; //device drivers
        std::map<int, yarp::dev::ITorqueControl*>       itrq;  // interface to read joint torques
        
        // input ports (the key of the maps is the sensor id)
        std::map<wbi::LocalId, yarp::os::BufferedPort<yarp::sig::Vector>*>   portsFTsens;
        std::map<wbi::LocalId, yarp::os::BufferedPort<yarp::sig::Vector>*>   portsIMU;
        std::map<wbi::LocalId, yarp::os::BufferedPort<yarp::sig::Vector>*>   portsTorqueSensor;
        
        bool openPwm(const int bodyPart);
        bool openEncoder(const int bodyPart);
        bool openImu(const wbi::LocalId &i);
        bool openFTsens(const wbi::LocalId &i);
        bool openTorqueSensor(const int bodyPart);
        
        // *** ENCODERS
        virtual bool addEncoder(const wbi::LocalId &j);
        virtual int addEncoders(const wbi::LocalIdList &j);
        // *** PWMs
        virtual bool addPwm(const wbi::LocalId &j);
        virtual int addPwms(const wbi::LocalIdList &j);
        // *** IMUs
        virtual bool addIMU(const wbi::LocalId &i);
        virtual int addIMUs(const wbi::LocalIdList &i);
        // *** FORCE/TORQUE SENSORS
        virtual bool addFTsensor(const wbi::LocalId &i);
        virtual int addFTsensors(const wbi::LocalIdList &i);
        // *** TORQUE SENSORS *** //
        virtual bool addTorqueSensor(const wbi::LocalId &i);
        virtual int addTorqueSensors(const wbi::LocalIdList &i);
        
        virtual bool readEncoder(const wbi::LocalId &i, double *q, double *stamps=0, bool wait=true);
        virtual bool readPwm(const wbi::LocalId &i, double *pwm, double *stamps=0, bool wait=true);
        virtual bool readIMU(const wbi::LocalId &i, double *inertial, double *stamps=0, bool wait=true);
        virtual bool readFTsensor(const wbi::LocalId &i, double *ftSens, double *stamps=0, bool wait=true);
        virtual bool readTorqueSensor(const wbi::LocalId &i, double *jointTorque, double *stamps=0, bool wait=true);

        virtual bool readEncoders(double *q, double *stamps=0, bool wait=true);
        virtual bool readPwms(double *pwm, double *stamps=0, bool wait=true);
        virtual bool readIMUs(double *inertial, double *stamps=0, bool wait=true);
        virtual bool readFTsensors(double *ftSens, double *stamps=0, bool wait=true);
        virtual bool readTorqueSensors(double *jointTorques, double *stamps=0, bool wait=true);

    public:
        // *** CONSTRUCTORS ***
        icubWholeBodySensors(const char* _name, const char* _robotName);

        /**
          * @param _name Local name of the interface (used as stem of port names)
          * @param _robotName Name of the robot
          * @param _bodyPartNames Vector of names of the body part (used when opening the polydrivers)
          * @param _ftSens_2_port List containing the port name for each force/torque sensor
          * @param _imu_2_port List containing the port name for each inertial measurement unit
          */
        icubWholeBodySensors(const char* _name, const char* _robotName, const std::vector<std::string> &_bodyPartNames, 
            const std::vector<id_2_PortName> &_ftSens_2_port, const std::vector<id_2_PortName> &_imu_2_port);

        virtual bool init();
        virtual bool close();

        /** Add the specified sensor so that it can be read. 
         * @param st Type of sensor.
         * @param sid Id of the sensor.
         * @return True if the sensor has been added, false otherwise (e.g. the sensor has been already added).
         */
        virtual bool addSensor(const wbi::SensorType st, const wbi::LocalId &sid);
        
        /** Add the specified sensors so that they can be read. 
         * @param st Type of sensors.
         * @param sids Ids of the sensors.
         * @return True if the sensor has been added, false otherwise (e.g. the sensor has been already added).
         */
        virtual int addSensors(const wbi::SensorType st, const wbi::LocalIdList &sids);

        /** Remove the specified sensor. 
         * @param st Type of the sensor to remove.
         * @param j Id of the sensor to remove.
         * @return True if the sensor has been removed, false otherwise.
         */
        virtual bool removeSensor(const wbi::SensorType st, const wbi::LocalId &sid);
        
        /** Get a copy of the sensor list of the specified sensor type.
         * @param st Type of sensors.
         * @return A copy of the sensor list. */
        virtual const wbi::LocalIdList& getSensorList(const wbi::SensorType st);
        
        /** Get the number of sensors of the specified type.
         * @return The number of sensors of the specified type. */
        virtual int getSensorNumber(const wbi::SensorType st);

        /** Read the specified sensor.
         * @param st Type of sensor to read.
         * @param sid Id of the sensor to read.
         * @param data Output data vector.
         * @param stamps Output vector of timestamps.
         * @param blocking If true, the reading is blocking, otherwise it is not.
         * @return True if all the readings succeeded, false otherwise.
         */
        virtual bool readSensor(const wbi::SensorType st, const wbi::LocalId &sid, double *data, double *stamps=0, bool blocking=true);
        
        /** Read all the sensors of the specified type.
         * @param st Type of the sensor to read.
         * @param sid Id of the sensor to read.
         * @param data Output data vector.
         * @param stamps Output vector of timestamps.
         * @param blocking If true, the reading is blocking, otherwise it is not.
         * @return True if the reading succeeded, false otherwise.
         */
        virtual bool readSensors(const wbi::SensorType st, double *data, double *stamps=0, bool blocking=true);
    };
    

    /*
     * Class for communicating with iCub's motor control boards.
     */
    class icubWholeBodyActuators : public wbi::iWholeBodyActuators
    {
    protected:
        bool                        initDone;       // true after init has been called, false before
        int                         dof;            // number of actuators considered
        std::string                 name;           // name used as root for the local ports
        std::string                 robot;          // name of the robot
        std::vector<int>            bodyParts;      // list of the body parts
        std::vector<std::string>    bodyPartNames;  // names of the body parts
        wbi::LocalIdList            jointIdList;    // list of the joint ids
        
        std::map<wbi::LocalId, wbi::ControlMode>        currentCtrlModes;    // current control mode of each actuator

        // yarp drivers
        std::map<int, yarp::dev::IPositionControl*>     ipos;
        std::map<int, yarp::dev::ITorqueControl*>       itrq;
        std::map<int, yarp::dev::IImpedanceControl*>    iimp;
        std::map<int, yarp::dev::IControlMode*>         icmd;
        std::map<int, yarp::dev::IVelocityControl2*>    ivel;
        std::map<int, yarp::dev::IOpenLoopControl*>     iopl;
        std::map<int, yarp::dev::PolyDriver*>           dd;
        
        /** Open the yarp PolyDriver relative to the specified body part. */
        bool openDrivers(int bodyPart);
        /** Convert the control modes defined in yarp/dev/IControlMode.h into the one defined in wbi. */
        wbi::ControlMode yarpToWbiCtrlMode(int yarpCtrlMode);
        /** Set the reference speed for the position control of the specified joint(s). */
        virtual bool setReferenceSpeed(double *rspd, int joint=-1);
        
    public:
        /** Constructor.
         * @param _name Name of this object, used as a stem for opening YARP ports.
         * @param _robot Name of the robot.
         * @param _bodyPartNames Vector containing the names of the body parts of iCub.
        */
        icubWholeBodyActuators(const char* _name, const char* _robot, const std::vector<std::string> &_bodyPartNames
            =std::vector<std::string>(iCub::skinDynLib::BodyPart_s,iCub::skinDynLib::BodyPart_s+sizeof(iCub::skinDynLib::BodyPart_s)/sizeof(std::string)));
        
        inline virtual ~icubWholeBodyActuators(){ close(); }
        virtual bool init();
        virtual bool close();

        inline virtual int getActuatorNumber(){ return dof; }
        virtual bool removeActuator(const wbi::LocalId &j);
        virtual bool addActuator(const wbi::LocalId &j);
        virtual int addActuators(const wbi::LocalIdList &j);
        inline virtual const wbi::LocalIdList& getActuatorList(){ return jointIdList; }
        
        /** Set the control mode of the specified joint(s).
          * @param controlMode Id of the control mode.
          * @param ref Reference value(s) for the controller.
          * @param joint Joint number, if negative, all joints are considered.
          * @return True if operation succeeded, false otherwise. */
        virtual bool setControlMode(wbi::ControlMode controlMode, double *ref=0, int joint=-1);
        
        /** Set the reference value for the controller of the specified joint(s).
          * @param ref Reference value(s) for the controller.
          * @param joint Joint number, if negative, all joints are considered.
          * @return True if operation succeeded, false otherwise. */
        virtual bool setControlReference(double *ref, int joint=-1);

        /** Set a parameter (e.g. a gain) of one or more joint controllers.
          * @param paramId Id of the parameter.
          * @param value Value(s) of the parameter.
          * @param joint Joint number, if negative, all joints are considered.
          * @return True if operation succeeded, false otherwise. */
        virtual bool setControlParam(wbi::ControlParam paramId, const void *value, int joint=-1);
    };
    

    /** 
     * Thread that estimates the state of the iCub robot. 
     */
    class icubWholeBodyEstimator: public yarp::os::RateThread
    {
    protected:
        icubWholeBodySensors        *sensors;
        //double                      estWind;        // time window for the estimation
        
        iCub::ctrl::AWLinEstimator  *dqFilt;        // joint velocity filter
        iCub::ctrl::AWQuadEstimator *d2qFilt;       // joint acceleration filter
        iCub::ctrl::AWLinEstimator  *dTauJFilt;     // joint torque derivative filter
        iCub::ctrl::AWLinEstimator  *dTauMFilt;     // motor torque derivative filter
        iCub::ctrl::FirstOrderLowPassFilter *tauJFilt;  ///< low pass filter for joint torque
        iCub::ctrl::FirstOrderLowPassFilter *tauMFilt;  ///< low pass filter for motor torque
        
        int dqFiltWL, d2qFiltWL;                    // window lengths of adaptive window filters
        double dqFiltTh, d2qFiltTh;                 // threshold of adaptive window filters
        int dTauMFiltWL, dTauJFiltWL;               // window lengths of adaptive window filters
        double dTauMFiltTh, dTauJFiltTh;            // threshold of adaptive window filters
        double tauJCutFrequency;
        double tauMCutFrequency;
        
        yarp::sig::Vector           q, qStamps;         // last joint position estimation
        yarp::sig::Vector           tauJ, tauJStamps;
        
        /* Resize all vectors using current number of DoFs. */
        void resizeAll(int n);
        void lockAndResizeAll(int n);

        /** Set the parameters of the adaptive window filter used for velocity estimation. */
        bool setVelFiltParams(int windowLength, double threshold);
        /** Set the parameters of the adaptive window filter used for acceleration estimation. */
        bool setAccFiltParams(int windowLength, double threshold);
        /** Set the parameters of the adaptive window filter used for joint torque derivative estimation. */
        bool setDtauJFiltParams(int windowLength, double threshold);
        /** Set the parameters of the adaptive window filter used for motor torque derivative estimation. */
        bool setDtauMFiltParams(int windowLength, double threshold);
        /** Set the cut frequency of the joint torque low pass filter. */
        bool setTauJCutFrequency(double fc);
        /** Set the cut frequency of the motor torque low pass filter. */
        bool setTauMCutFrequency(double fc);
        
    public:
        
        yarp::os::Semaphore         mutex;          // mutex for access to class global variables
        
        // the elements of this struct are accessed by the state interface
        // the state interface takes the mutex before accessing this struct
        struct 
        {
            yarp::sig::Vector lastQ;                    // last joint position estimation
            yarp::sig::Vector lastDq;                   // last joint velocity estimation
            yarp::sig::Vector lastD2q;                  // last joint acceleration estimation
            yarp::sig::Vector lastTauJ;                 // last joint torque
            yarp::sig::Vector lastTauM;                 // last motor torque
            yarp::sig::Vector lastDtauJ;                // last joint torque derivative
            yarp::sig::Vector lastDtauM;                // last motor torque derivative
        } 
        estimates;

        /** Constructor. */
        icubWholeBodyEstimator(int _period, icubWholeBodySensors *_sensors);
        
        bool lockAndSetEstimationParameter(const wbi::EstimateType et, const wbi::EstimationParameter ep, const void *value);

        bool threadInit();
        void run();
        void threadRelease();
        
        /** Take the mutex and copy the content of src into dest. */
        bool lockAndCopyVector(const yarp::sig::Vector &src, double *dest);
        /** Take the mutex and copy the i-th element of src into dest. */
        bool lockAndCopyVectorElement(int i, const yarp::sig::Vector &src, double *dest);
    };
    

    /**
     * Class to access the estimates of the states of iCub.
     */
    class icubWholeBodyStates : public wbi::iWholeBodyStates
    {
    protected:
        icubWholeBodySensors        *sensors;       // interface to access the robot sensors
        icubWholeBodyEstimator      *estimator;     // estimation thread
        wbi::LocalIdList            emptyList;      ///< empty list of IDs to return in case of error
        //double                      estWind;      // time window for the estimation
        
        virtual bool lockAndReadSensor(const wbi::SensorType st, const wbi::LocalId sid, double *data, double time, bool blocking);
        virtual bool lockAndReadSensors(const wbi::SensorType st, double *data, double time, bool blocking);
        virtual bool lockAndAddSensor(const wbi::SensorType st, const wbi::LocalId &sid);
        virtual int lockAndAddSensors(const wbi::SensorType st, const wbi::LocalIdList &sids);
        virtual bool lockAndRemoveSensor(const wbi::SensorType st, const wbi::LocalId &sid);
        virtual wbi::LocalIdList lockAndGetSensorList(const wbi::SensorType st);
        virtual int lockAndGetSensorNumber(const wbi::SensorType st);

        /** Get the velocity of the specified motor. */
        bool getMotorVel(const wbi::LocalId &sid, double *data, double time, bool blocking);
        /** Get the velocities of all the robot motors. */
        bool getMotorVel(double *data, double time, bool blocking);
        
    public:
        // *** CONSTRUCTORS ***
        icubWholeBodyStates(const char* _name, const char* _robotName, double estimationTimeWindow);
        inline virtual ~icubWholeBodyStates(){ close(); }
        
        virtual bool init();
        virtual bool close();
        
        /** Add the specified estimate so that it can be read. 
         * @param st Type of estimate.
         * @param sid Id of the estimate.
         * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
         */
        virtual bool addEstimate(const wbi::EstimateType st, const wbi::LocalId &sid);
        
        /** Add the specified estimates so that they can be read. 
         * @param st Type of estimates.
         * @param sids Ids of the estimates.
         * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
         */
        virtual int addEstimates(const wbi::EstimateType st, const wbi::LocalIdList &sids);

        /** Remove the specified estimate. 
         * @param st Type of the estimate to remove.
         * @param j Id of the estimate to remove.
         * @return True if the estimate has been removed, false otherwise.
         */
        virtual bool removeEstimate(const wbi::EstimateType st, const wbi::LocalId &sid);
        
        /** Get a copy of the estimate list of the specified estimate type.
         * @param st Type of estimate.
         * @return A copy of the estimate list. */
        virtual const wbi::LocalIdList& getEstimateList(const wbi::EstimateType st);
        
        /** Get the number of estimates of the specified type.
         * @return The number of estimates of the specified type. */
        virtual int getEstimateNumber(const wbi::EstimateType st);

        /** Get the estimate of the specified quantity at the specified time.
         * @param et Type of estimate to get.
         * @param sid Id of the estimate
         * @param data Output data vector.
         * @param time Time at which to estimate the quantity.
         * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
         * @return True if all the estimate succeeded, false otherwise.
         */
        virtual bool getEstimate(const wbi::EstimateType et, const wbi::LocalId &sid, double *data, double time=-1.0, bool blocking=true);

        /** Get all the estimates of the specified estimate type at the specified time.
         * @param et Type of estimate to get.
         * @param data Output data vector.
         * @param time Time at which to estimate the quantity.
         * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
         * @return True if all the estimate succeeded, false otherwise.
         */
        virtual bool getEstimates(const wbi::EstimateType et, double *data, double time=-1.0, bool blocking=true);

        /** Set the value of the specified parameter of the estimation algorithm
         * of the specified estimate type.
         * @param et Estimation type (e.g. joint velocity, motor torque).
         * @param ep Parameter to set.
         * @param value Value of the parameter to set.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool setEstimationParameter(const wbi::EstimateType et, const wbi::EstimationParameter ep, const void *value);
    };
    

    /**
     * Interface to the kinematic/dynamic model of iCub.
     */
    class icubWholeBodyModel: public wbi::iWholeBodyModel
    {
    protected:
        wbi::LocalIdList jointIdList;
        int dof;
        iCub::iDynTree::iCubTree * p_icub_model;
        iCub::iDynTree::iCubTree_version_tag version;
        
        yarp::sig::Matrix world_base_transformation;
        
        yarp::sig::Vector v_base, a_base;
        yarp::sig::Vector omega_base, domega_base;
        
        yarp::sig::Vector all_q;
        yarp::sig::Vector all_dq;
        yarp::sig::Vector all_ddq;
        
        yarp::sig::Vector all_q_min, all_q_max;

        // *** Variables needed for opening IControlLimits interfaces
        std::string                                 name;           // name used as root for the local ports
        std::string                                 robot;          // name of the robot
        std::vector<int>                            bodyParts;      // list of the body parts
        std::vector<std::string>                    bodyPartNames;  // names of the body parts
        std::map<int, yarp::dev::PolyDriver*>       dd;
        std::map<int, yarp::dev::IControlLimits*>   ilim;

        bool openDrivers(int bp);
        
        bool convertBasePose(const wbi::Frame &xBase, yarp::sig::Matrix & H_world_base);
        bool convertBaseVelocity(const double *dxB, yarp::sig::Vector & v_b, yarp::sig::Vector & omega_b);
        bool convertBaseAcceleration(const double *ddxB, yarp::sig::Vector & a_b, yarp::sig::Vector & domega_b);
        
        bool convertQ(const double *q_input, yarp::sig::Vector & q_complete_output);
        bool convertQ(const yarp::sig::Vector & q_complete_input, double *q_output);
        bool convertDQ(const double *dq_input, yarp::sig::Vector & dq_complete_output);
        bool convertDDQ(const double *ddq_input, yarp::sig::Vector & ddq_complete_output);
        
    public:
         // *** CONSTRUCTORS ***
        /**
          * @param _name Local name of the interface (used as stem of port names)
          * @param _robotName Name of the robot
          * @param head_version the version of the head of the iCub (1 or 2, default: 2)
          * @param legs_version the version of the legs of the iCub (1 or 2, default: 1)
          * @param initial_q the initial value for all the 32 joint angles (default: all 0)
          * @param _bodyPartNames Vector of names of the body part (used when opening the polydrivers)
          */
        icubWholeBodyModel(const char* _name, const char* _robotName, int head_version=2, int legs_version=1, double* initial_q=0,
            const std::vector<std::string> &_bodyPartNames=std::vector<std::string>(iCub::skinDynLib::BodyPart_s,iCub::skinDynLib::BodyPart_s+sizeof(iCub::skinDynLib::BodyPart_s)/sizeof(std::string)));
        
        inline virtual ~icubWholeBodyModel(){ close(); }
        virtual bool init();
        virtual bool close();

        inline virtual int getDoFs(){ return dof; }

        /** Remove the specified joint form the robot model. The joint is considered blocked
          * at its last known value (values of the joint angles are stored whenever a method of 
          * iWholeBodyModel is called). If no previous value of the joint angle is known, zero is assumed.
          * @param j Id of the joint to remove
          * @return True if the joint was found and removed, false otherwise. */
        virtual bool removeJoint(const wbi::LocalId &j);
        virtual bool addJoint(const wbi::LocalId &j);
        virtual int addJoints(const wbi::LocalIdList &j);
        virtual const wbi::LocalIdList& getJointList(){    return jointIdList; }
        
        virtual bool getJointLimits(double *qMin, double *qMax, int joint=-1);

        /** Get the id of the link with the specified name.
          * @param linkName Name of the link.
          * @param linkId Id of the link (if found).
          * @return True if the link name was found, false otherwise. */
        inline virtual bool getLinkId(const char *linkName, int &linkId)
        { linkId = p_icub_model->getLinkIndex(linkName); return linkId>=0; }
        
        /** Compute rototranslation matrix from root reference frame to reference frame associated to the specified link.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link that is the target of the rototranslation
          * @param H Output 4x4 rototranslation matrix (stored by rows)
          * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeH(double *q, const wbi::Frame &xBase, int linkId, wbi::Frame &H);
        
        /** Compute the Jacobian of the specified point of the robot.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link
          * @param J Output 6xN Jacobian matrix (stored by rows), where N=number of joints
          * @param pos 3d position of the point expressed w.r.t the link reference frame
          * @return True if the operation succeeded, false otherwise (invalid input parameters) 
          * @note If linkId==COM_LINK_ID then the angular part of J is related to the angular velocity of the
          *       whole multi-body system. This Jacobian premultiplied by the whole robot's 6D inertia
          *       matrix is equal to the Jacobian of the angular momentum of the whole robot. */
        virtual bool computeJacobian(double *q, const wbi::Frame &xBase, int linkId, double *J, double *pos=0);
        
        /** Given a point on the robot, compute the product between the time derivative of its 
          * Jacobian and the joint velocity vector.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param dq Joint velocities
          * @param linkId Id of the link
          * @param dJdq Output 6-dim vector containing the product dJ*dq 
          * @param pos 3d position of the point expressed w.r.t the link reference frame
          * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeDJdq(double *q, const wbi::Frame &xBase, double *dq, double *dxB, int linkId, double *dJdq, double *pos=0);
        
        /** Compute the forward kinematics of the specified joint.
          * @param q Joint angles.
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link.
          * @param x Output 7-dim pose vector (3 for pos, 4 for quaternion orientation).
          * @return True if the operation succeeded, false otherwise. */
        virtual bool forwardKinematics(double *q, const wbi::Frame &xBase, int linkId, double *x);
        
        /** Compute the inverse dynamics.
          * @param q Joint angles.
          * @param xBase Rototranslation from world frame to robot base frame
          * @param dq Joint velocities.
          * @param dxB Velocity of the robot base, 3 values for linear velocity and 3 values for angular velocity.
          * @param ddq Joint accelerations.
          * @param ddxB Acceleration of the robot base, 3 values for linear acceleration and 3 values for angular acceleration.
          * @param tau Output joint torques.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool inverseDynamics(double *q, const wbi::Frame &xBase, double *dq, double *dxB, double *ddq, double *ddxB, double *tau);

        /** Compute the direct dynamics.
         * @param q Joint angles.
         * @param xBase Rototranslation from world frame to robot base frame
         * @param dq Joint velocities.
         * @param dxB Velocity of the robot base, 3 values for linear velocity and 3 values for angular velocity.
         * @param M Output N+6xN+6 mass matrix, with N=number of joints.
         * @param h Output N+6-dim vector containing all generalized bias forces (gravity+Coriolis+centrifugal).
         * @return True if the operation succeeded, false otherwise. */
        virtual bool directDynamics(double *q, const wbi::Frame &xBase, double *dq, double *dxB, double *M, double *h);
    };
    

    const int JOINT_ESTIMATE_TYPES_SIZE = 3;
    ///< estimate types that are automatically added when calling addJoint(s) and automatically removed when calling removeJoint
    const wbi::EstimateType jointEstimateTypes[JOINT_ESTIMATE_TYPES_SIZE] =
    {
        wbi::ESTIMATE_JOINT_POS,         // joint position
        //wbi::ESTIMATE_JOINT_VEL,         // joint velocity
        //wbi::ESTIMATE_JOINT_ACC,         // joint acceleration
        wbi::ESTIMATE_JOINT_TORQUE,      // joint torque
        //wbi::ESTIMATE_MOTOR_VEL,         // motor velocity
        //wbi::ESTIMATE_MOTOR_TORQUE,      // motor torque
        wbi::ESTIMATE_MOTOR_PWM,         // motor PWM (proportional to motor voltage)
    };
    
    /**
     * Class to communicate with iCub.
     */
    class icubWholeBodyInterface : public wbi::wholeBodyInterface
    {
    protected:
        icubWholeBodyStates     *stateInt;
        icubWholeBodyActuators  *actuatorInt;
        icubWholeBodyModel      *modelInt;
       
    public:
        // *** CONSTRUCTORS ***
        icubWholeBodyInterface(const char* _name, const char* _robotName, int head_version=2, int legs_version=1);
        
        inline virtual ~icubWholeBodyInterface(){ close(); }
        virtual bool init();
        virtual bool close();
        virtual bool removeJoint(const wbi::LocalId &j);
        virtual bool addJoint(const wbi::LocalId &j);
        virtual int addJoints(const wbi::LocalIdList &j);
   
        // ACTUATORS
        virtual int getActuatorNumber(){                        return actuatorInt->getActuatorNumber(); }
        virtual bool removeActuator(const wbi::LocalId &j){     return actuatorInt->removeActuator(j); }
        virtual bool addActuator(const wbi::LocalId &j){        return actuatorInt->addActuator(j); }
        virtual int addActuators(const wbi::LocalIdList &j){    return actuatorInt->addActuators(j); }
        virtual const wbi::LocalIdList& getActuatorList(){      return actuatorInt->getActuatorList(); }
        virtual bool setControlMode(wbi::ControlMode cm, double *ref=0, int jnt=-1)
        { return actuatorInt->setControlMode(cm, ref, jnt); }
        virtual bool setControlReference(double *ref, int jnt=-1)
        { return actuatorInt->setControlReference(ref, jnt); }
        virtual bool setControlParam(wbi::ControlParam parId, const void *val, int jnt=-1)
        { return actuatorInt->setControlParam(parId, val, jnt); }

        // STATES
        virtual bool addEstimate(const wbi::EstimateType st, const wbi::LocalId &sid){      return stateInt->addEstimate(st, sid); }
        virtual int addEstimates(const wbi::EstimateType st, const wbi::LocalIdList &sids){ return stateInt->addEstimates(st, sids); }
        virtual bool removeEstimate(const wbi::EstimateType st, const wbi::LocalId &sid){   return stateInt->removeEstimate(st, sid); }
        virtual const wbi::LocalIdList& getEstimateList(const wbi::EstimateType st){        return stateInt->getEstimateList(st); }
        virtual int getEstimateNumber(const wbi::EstimateType st){                          return stateInt->getEstimateNumber(st); }
        virtual bool getEstimate(const wbi::EstimateType et, const wbi::LocalId &sid, double *data, double time=-1.0, bool blocking=true)
        { return stateInt->getEstimate(et, sid, data, time, blocking); }
        virtual bool getEstimates(const wbi::EstimateType et, double *data, double time=-1.0, bool blocking=true)
        { return stateInt->getEstimates(et, data, time, blocking); }
        virtual bool setEstimationParameter(const wbi::EstimateType et, const wbi::EstimationParameter ep, const void *value)
        { return stateInt->setEstimationParameter(et, ep, value); }
        
        // MODEL
        virtual int getDoFs(){ return modelInt->getDoFs(); }
        virtual const wbi::LocalIdList& getJointList(){ return modelInt->getJointList(); }
        virtual bool getLinkId(const char *linkName, int &linkId)
        { return modelInt->getLinkId(linkName, linkId); }
        virtual bool getJointLimits(double *qMin, double *qMax, int joint=-1)
        { return modelInt->getJointLimits(qMin, qMax, joint); }
        virtual bool computeH(double *q, const wbi::Frame &xB, int linkId, wbi::Frame &H)
        { return modelInt->computeH(q, xB, linkId, H); }
        virtual bool computeJacobian(double *q, const wbi::Frame &xB, int linkId, double *J, double *pos=0)
        { return modelInt->computeJacobian(q, xB, linkId, J, pos); }
        virtual bool computeDJdq(double *q, const wbi::Frame &xB, double *dq, double *dxB, int linkId, double *dJdq, double *pos=0)
        { return modelInt->computeDJdq(q, xB, dq, dxB, linkId, dJdq, pos); }
        virtual bool forwardKinematics(double *q, const wbi::Frame &xB, int linkId, double *x)
        { return modelInt->forwardKinematics(q, xB, linkId, x); }
        virtual bool inverseDynamics(double *q, const wbi::Frame &xB, double *dq, double *dxB, double *ddq, double *ddxB, double *tau)
        { return modelInt->inverseDynamics(q, xB, dq, dxB, ddq, ddxB, tau); }
        virtual bool directDynamics(double *q, const wbi::Frame &xB, double *dq, double *dxB, double *M, double *h)
        { return modelInt->directDynamics(q, xB, dq, dxB, M, h); }
    };
    
} // end namespace wbiIcub

#endif

