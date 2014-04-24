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

#ifndef WBSTATESLOCAL_ICUB_H
#define WBSTATESLOCAL_ICUB_H

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <iCub/iDynTree/iCubTree.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <wbiIcub/wbiIcubUtil.h>
#include <map>


namespace wbiIcub
{
    /**
     * Enum of iCub subtrees
     *
     */
    enum iCubSubtree {
        TORSO_SUBTREE,
        LEFT_ARM_SUBTREE,
        RIGHT_ARM_SUBTREE,
        LEFT_LEG_SUBTREE,
        RIGHT_LEG_SUBTREE,
        LEFT_FOOT_SUBTREE,
        RIGHT_FOOT_SUBTREE
    };


    //< \todo TODO make SKIN_EVENTS_TIMEOUT a proper parameter
    #define SKIN_EVENTS_TIMEOUT 0.2     // max time (in sec) a contact is kept without reading anything from the skin events port
     /**
     * Thread that estimates the dynamic state of the iCub robot.
     */
    class icubWholeBodyDynamicsEstimator: public yarp::os::RateThread
    {
    protected:
        icubWholeBodySensors        *sensors;
        yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> * port_skin_contacts;
        iCub::iDynTree::iCubTree * icub_model;

        //double                      estWind;        // time window for the estimation

        iCub::ctrl::AWLinEstimator  *dqFilt;        // joint velocity filter
        iCub::ctrl::AWQuadEstimator *d2qFilt;       // joint acceleration filter
        iCub::ctrl::AWLinEstimator  *dTauJFilt;     // joint torque derivative filter
        iCub::ctrl::AWLinEstimator  *dTauMFilt;     // motor torque derivative filter
        iCub::ctrl::FirstOrderLowPassFilter *tauJFilt;  ///< low pass filter for joint torque
        iCub::ctrl::FirstOrderLowPassFilter *tauMFilt;  ///< low pass filter for motor torque
        iCub::ctrl::FirstOrderLowPassFilter *pwmFilt;   ///< low pass filter for motor PWM
        std::vector<iCub::ctrl::FirstOrderLowPassFilter *> imuLinearAccelerationFilters; ///<  low pass filters for IMU linear accelerations
        std::vector<iCub::ctrl::FirstOrderLowPassFilter *> imuAngularVelocityFilters; ///< low pass filters for IMU angular velocity
        std::vector<iCub::ctrl::FirstOrderLowPassFilter *> imuMagnetometerFilters; ///< low pass filters for IMU magnetometer
        std::vector<iCub::ctrl::FirstOrderLowPassFilter *> forcetorqueFilters; ///< low pass filters for ForceTorque sensors

        iCub::ctrl::AWLinEstimator * imuAngularAccelerationFilt;

        int dqFiltWL, d2qFiltWL;                    // window lengths of adaptive window filters
        double dqFiltTh, d2qFiltTh;                 // threshold of adaptive window filters
        int dTauMFiltWL, dTauJFiltWL;               // window lengths of adaptive window filters
        double dTauMFiltTh, dTauJFiltTh;            // threshold of adaptive window filters
        double tauJCutFrequency;
        double tauMCutFrequency;
        double pwmCutFrequency;
        double imuLinearAccelerationCutFrequency;
        double imuAngularVelocityCutFrequency;
        double imuMagnetometerCutFrequency;
        double forcetorqueCutFrequency;

        double imuAngularAccelerationFiltTh;
        int imuAngularAccelerationFiltWL;



        yarp::sig::Vector           q, qStamps;         // last joint position estimation
        yarp::sig::Vector           tauJ, tauJStamps;
        yarp::sig::Vector           pwm, pwmStamps;

        std::vector<yarp::sig::Vector> forcetorques;
        yarp::sig::Vector forcetorquesStamps;

        std::vector<yarp::sig::Vector> forcetorques_offset;

        std::vector<yarp::sig::Vector> IMUs;
        yarp::sig::Vector IMUStamps;

        //Data structures related to skin
        iCub::skinDynLib::skinContactList skinContacts;
        double skin_contact_listStamp;
        double last_reading_skin_contact_list_Stamp;

        iCub::skinDynLib::dynContactList dynContacts;

        //Data structures related to end effector wrenches
        yarp::sig::Vector left_gripper_ee_wrench;
        yarp::sig::Vector right_gripper_ee_wrench;
        yarp::sig::Vector left_sole_ee_wrench;
        yarp::sig::Vector right_sole_ee_wrench;

        yarp::sig::Vector left_hand_ee_wrench;
        yarp::sig::Vector right_hand_ee_wrench;
        yarp::sig::Vector left_foot_ee_wrench;
        yarp::sig::Vector right_foot_ee_wrench;

        int left_hand_link_id;
        int right_hand_link_id;
        int left_foot_link_id;
        int right_foot_link_id;

        int left_gripper_frame_id;
        int right_gripper_frame_id;
        int left_sole_frame_id;
        int right_sole_frame_id;

        bool left_arm_ee_contact_found;
        bool right_arm_ee_contact_found;
        bool left_leg_ee_contact_found;
        bool right_leg_ee_contact_found;


        //Data structures related to IMU used for dynamical model
        bool enable_omega_domega_IMU;

        yarp::sig::Vector omega_used_IMU;
        yarp::sig::Vector domega_used_IMU;
        yarp::sig::Vector ddp_used_IMU;

        /* Resize all vectors using current number of DoFs. */
        void resizeAll(int n);
        void lockAndResizeAll(int n);

        //< \todo TODO add general interface using type (?) of sensors

        /* Resize all FT sensors related vectors using current number of Force Torque sensors */
        void resizeFTs(int n);
        void lockAndResizeFTs(int n);

        /* Resize all IMU sensors related vectors using current number of IMU sensors */
        void resizeIMUs(int n);
        void lockAndResizeIMUs(int n);

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
        /** Set the cut frequency of the motor PWM low pass filter. */
        bool setPwmCutFrequency(double fc);
        /** Enable or disable the use of IMU angular velocity and acceleration in external force estimation */
        bool setEnableOmegaDomegaIMU(bool opt);

        /** Read the skin contacts and generated the contact points for external wrenches  estimation */
        void readSkinContacts();

        /** For a given subtree, get the default contact point (the one used if there are now contacts coming from the skin */
        iCub::skinDynLib::dynContact getDefaultContact(const iCubSubtree icub_subtree);


        /**  Estimate internal torques and external forces from measured sensors, using iDynTree library */
        void estimateExternalForcesAndJointTorques();

        /** Store external wrenches ad the end effectors */
        void readEndEffectorsExternalWrench();

        /** Version of considered iCub robot */
        iCub::iDynTree::iCubTree_version_tag icub_version;

    public:

        yarp::os::Semaphore         mutex;          // mutex for access to class global variables
        yarp::os::Semaphore         model_mutex;    // mutex for access the dynamic model
        yarp::os::Semaphore         run_mutex;      // mutex for avoiding multiple run being execute together

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
            yarp::sig::Vector lastPwm;                  // last motor PWM
            std::vector<yarp::sig::Vector> lastForceTorques; //last Force/torques sensors estimation
            std::vector<yarp::sig::Vector> lastIMUs;    //last IMU sensors estimation
        }
        estimates;

        iCub::skinDynLib::dynContactList estimatedLastDynContacts;
        iCub::skinDynLib::skinContactList estimatedLastSkinDynContacts;

         /** Constructor.
         *
         * @param port_skin_contacts pointer to a port reading a skinContactList from the robot skin
         * \todo TODO skin_contacts should be read from the WholeBodySensors interface
         */
        icubWholeBodyDynamicsEstimator(int _period,
                                       icubWholeBodySensors *_sensors,
                                       yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> * _port_skin_contacts,
                                       iCub::iDynTree::iCubTree_version_tag icub_version);

        bool lockAndSetEstimationParameter(const wbi::EstimateType et, const wbi::EstimationParameter ep, const void *value);

        bool lockAndSetEstimationOffset(const wbi::EstimateType et, const wbi::LocalId & sid, const double *value);
        bool lockAndGetEstimationOffset(const wbi::EstimateType et, const wbi::LocalId & sid, double *value);


        bool threadInit();
        void run();
        void threadRelease();

        /** Take the mutex and copy the content of src into dest. */
        bool lockAndCopyVector(const yarp::sig::Vector &src, double *dest);
        /** Take the mutex and copy the i-th element of src into dest. */
        bool lockAndCopyVectorElement(int i, const yarp::sig::Vector &src, double *dest);
        /** Take the mutex and copy the (serialized) content of src into dest */
        bool lockAndCopyVectorOfVectors(const std::vector<yarp::sig::Vector> &src, double *dest);
        /** Take the mutex and copy the i-th Vector of a vector<Vector> of src into dest */
        bool lockAndCopyElementVectorFromVector(int i, const std::vector<yarp::sig::Vector> &src, double *dest);

        /** Take the mutex and copy the external force/torque acting on link sid */


    };




     /**
     * Class to access the estimates, by doing a local estimation
     */
    class icubWholeBodyStatesLocal : public wbi::iWholeBodyStates
    {
    protected:
        icubWholeBodySensors                *sensors;       // interface to access the robot sensors
        icubWholeBodyDynamicsEstimator      *estimator;     // estimation thread
        yarp::os::BufferedPort<iCub::skinDynLib::skinContactList>                *skin_contacts_port; //port to the skin contacts
        wbi::LocalIdList                    emptyList;      ///< empty list of IDs to return in case of error
        //double                      estWind;      // time window for the estimation

        virtual bool lockAndReadSensor(const wbi::SensorType st, const wbi::LocalId sid, double *data, double time, bool blocking);
        virtual bool lockAndReadSensors(const wbi::SensorType st, double *data, double time, bool blocking);
        virtual bool lockAndAddSensor(const wbi::SensorType st, const wbi::LocalId &sid);
        virtual int lockAndAddSensors(const wbi::SensorType st, const wbi::LocalIdList &sids);
        virtual bool lockAndRemoveSensor(const wbi::SensorType st, const wbi::LocalId &sid);
        virtual wbi::LocalIdList lockAndGetSensorList(const wbi::SensorType st);
        virtual int lockAndGetSensorNumber(const wbi::SensorType st);

        bool lockAndReadExternalForces(iCub::skinDynLib::skinContactList & external_forces_list);
        bool lockAndReadExternalForceTorque();


        /** Get the velocity of the specified motor. */
        bool getMotorVel(const wbi::LocalId &sid, double *data, double time, bool blocking);
        /** Get the velocities of all the robot motors. */
        bool getMotorVel(double *data, double time, bool blocking);


    public:
        // *** CONSTRUCTORS ***
        icubWholeBodyStatesLocal(const char* _name, const char* _robotName, iCub::iDynTree::iCubTree_version_tag icub_version);
        inline virtual ~icubWholeBodyStatesLocal(){ close(); }

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

        /////////////////////////////////////////////////////
        ///< Implementation specific methods
        /////////////////////////////////////////////////////
        bool setEstimationOffset(const wbi::EstimateType et, const wbi::LocalId & sid, const double *value);

        bool getEstimationOffset(const wbi::EstimateType et, const wbi::LocalId & sid, double *value);


        /** Get the estimated external force/torques
         *
         * \note temporary interface, should be substituted by properly defining an external force/torque estimate
         * @param external_forces_list list of estimated external wrenches
         * @return True if the operation succeeded, false otherwise.
         */
        bool getEstimatedExternalForces(iCub::skinDynLib::skinContactList & external_forces_list);

    };
}

#endif
