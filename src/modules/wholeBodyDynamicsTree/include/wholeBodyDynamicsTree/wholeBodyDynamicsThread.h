/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#ifndef WHOLE_BODY_DYNAMICS_THREAD
#define WHOLE_BODY_DYNAMICS_THREAD

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Mutex.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IInteractionMode.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <wbi/wbi.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarpWholeBodyInterface/yarpWholeBodyStatesLocal.h>

class iCubTreeStatus
{
public:
    yarp::sig::Vector q;
    yarp::sig::Vector dq;
    yarp::sig::Vector ddq;
    yarp::sig::Vector omega_imu;
    yarp::sig::Vector domega_imu;
    yarp::sig::Vector proper_ddp_imu;
    yarp::sig::Vector wbi_imu;
    std::vector<yarp::sig::Vector> measured_ft_sensors;
    std::vector<yarp::sig::Vector> estimated_ft_sensors;

    iCubTreeStatus(int nrOfDOFs=0, int nrOfFTSensors=0);
    bool setNrOfDOFs(int nrOfDOFs);
    bool setNrOfFTSensors(int nrOfFTSensors);
    bool zero();
};

struct outputTorquePortInformation
{
    std::string port_name;
    int magic_number;
    std::vector< int > wbi_numeric_ids_to_publish;
    yarp::sig::Vector output_vector;
    yarp::os::BufferedPort<yarp::os::Bottle> * output_port;
};

/**
 *
  */
class wholeBodyDynamicsThread: public yarp::os::RateThread
{
    std::string name;
    std::string robotName;
    yarpWbi::yarpWholeBodyStatesLocal *estimator;

    int                 printCountdown;         // every time this is 0 (i.e. every PRINT_PERIOD ms) print stuff
    double              PRINT_PERIOD;

    enum { NORMAL, CALIBRATING, CALIBRATING_ON_DOUBLE_SUPPORT } wbd_mode;     /// < Mode of operation of the thread: normal operation or calibration

    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_wrench_RA;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_wrench_LA;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_wrench_RL;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_wrench_LL;

    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_cartesian_wrench_RA;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_cartesian_wrench_LA;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_cartesian_wrench_RL;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_cartesian_wrench_LL;

    yarp::os::BufferedPort<yarp::sig::Vector> *port_sensor_wrench_RL;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_sensor_wrench_LL;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_model_wrench_RL;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_model_wrench_LL;


    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> *port_contacts;

    /*
    yarp::os::BufferedPort<yarp::sig::Vector> *port_all_accelerations;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_all_velocities;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_all_positions;
    */

    // ports outputing the external dynamics seen at the F/T sensor
    /*
    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_ft_arm_left;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_ft_arm_right;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_ft_leg_left;
    yarp::os::BufferedPort<yarp::sig::Vector> *port_external_ft_leg_right;
    */

    yarp::os::BufferedPort<yarp::sig::Vector> * port_icubgui_base;

    yarp::os::BufferedPort<yarp::sig::Vector> * port_filtered_inertial;

    std::vector<yarp::os::BufferedPort<yarp::sig::Vector> *> port_filtered_ft;


    yarp::os::Stamp timestamp;

    template <class T> void broadcastData(T& _values, yarp::os::BufferedPort<T> *_port);
    void closePort(yarp::os::Contactable *_port);
    void writeTorque(yarp::sig::Vector _values, int _address, yarp::os::BufferedPort<yarp::os::Bottle> *_port);
    void publishTorques();
    void publishContacts();
    void getEndEffectorWrenches();
    void publishEndEffectorWrench();
    void publishBaseToGui();
    void publishFilteredInertialForGravityCompensator();
    void publishFilteredFTWithoutOffset();
    void publishAnkleFootForceTorques();
    bool decodeCalibCode(const std::string calib_code);
    void disableCalibration();


    wbi::ID convertFTiDynTreeToFTwbi(int ft_sensor_id);
    void normal_run();
    void calibration_run();
    void calibration_on_double_support_run();


    //Buffer vectors
    yarp::sig::Vector all_torques;

    //Data structures for mapping between wbi and output ports
    // this are populated by the WBD_TORQUE_PORTS group
    std::vector< outputTorquePortInformation > output_torque_ports;

    iCub::skinDynLib::skinContactList external_forces_list;

    yarp::sig::Vector LAExternalWrench;
    yarp::sig::Vector RAExternalWrench;
    yarp::sig::Vector LLExternalWrench;
    yarp::sig::Vector RLExternalWrench;

    yarp::sig::Vector LACartesianExternalWrench;
    yarp::sig::Vector RACartesianExternalWrench;
    yarp::sig::Vector LLCartesianExternalWrench;
    yarp::sig::Vector RLCartesianExternalWrench;

    yarp::sig::Vector iCubGuiBase;
    yarp::sig::Vector FilteredInertialForGravityComp;

    //Calibration related variables
    yarp::os::Mutex run_mutex;
    yarp::os::Mutex calibration_mutex;
    iCubTreeStatus tree_status;

    iCub::iDynTree::TorqueEstimationTree * icub_model_calibration; 
    std::string calibration_support_link;

    int samples_requested_for_calibration;
    int max_samples_for_calibration;

    std::vector<int> arms_fts;
    std::vector<int> legs_fts;
    std::vector<int> feet_fts;

    int left_hand_link_id;
    int right_hand_link_id;
    int left_foot_link_id;
    int right_foot_link_id;

    int left_gripper_frame_id;
    int right_gripper_frame_id;
    int left_sole_frame_id;
    int right_sole_frame_id;

    int left_hand_link_idyntree_id;
    int right_hand_link_idyntree_id;
    int left_foot_link_idyntree_id;
    int right_foot_link_idyntree_id;

    int root_link_idyntree_id;

    int left_gripper_frame_idyntree_id;
    int right_gripper_frame_idyntree_id;
    int left_sole_frame_idyntree_id;
    int right_sole_frame_idyntree_id;

    yarp::sig::Matrix transform_mat_buffer;

    int samples_used_for_calibration;

    std::vector<bool> calibrate_ft_sensor;
    std::vector<yarp::sig::Vector> offset_buffer;
    //End of Calibration related variables

    bool assume_fixed_base_calibration;
    std::string fixed_link_calibration;

    //zmp test mode
    bool zmp_test_mode;
    enum { LEFT_FOOT, RIGHT_FOOT } foot_under_zmp_test;
    int ankle_joint_idyntree_id;
    int foot_sole_fake_joint_idyntree_id;
    int foot_sole_link_idyntree_id;

   // port that output the wrenches relative to the foot/ankle
    yarp::os::BufferedPort<yarp::sig::Vector> * port_joint_ankle_cartesian_wrench;
    yarp::os::BufferedPort<yarp::sig::Vector> * port_joint_ankle_cartesian_wrench_from_model;
    yarp::os::BufferedPort<yarp::sig::Vector> * port_joint_foot_cartesian_wrench;
    yarp::os::BufferedPort<yarp::sig::Vector> * port_joint_foot_cartesian_wrench_from_model;
    iCub::iDynTree::TorqueEstimationTree * icub_model_zmp;

    bool autoconnect;

    bool publish_filtered_ft;

    yarp::os::Property yarp_options;

    /// attributes useful for setting the interaction mode to stiff
    /// when the wholeBodyDynamicsTree is closing
    struct {
        std::vector<std::string>           controlBoardNames;
        // list of controlBoard/Axis pair for each joint
        std::vector< std::pair<int,int> >  controlBoardAxisList;
        std::vector<yarp::dev::PolyDriver *> deviceDrivers;
        std::vector<yarp::dev::IControlMode2 *> controlModeInterfaces;
        std::vector<yarp::dev::IInteractionMode *> interactionModeInterfaces;

    } torqueEstimationControlBoards;

public:

    wholeBodyDynamicsThread(std::string _name,
                            std::string _robotName,
                            int _period,
                            yarpWbi::yarpWholeBodyStatesLocal *_wbi,
                            yarp::os::Property & yarpWbiOptions,
                            bool _autoconnect,
                            bool assume_fixed_base_calibration,
                            std::string fixed_link,
                            bool zmp_test_mode, std::string foot_to_test,
                            bool publish_filtered_ft
                           );

    bool threadInit();
    bool calibrateOffset(const std::string calib_code, const int nr_of_samples );
    bool calibrateOffsetOnDoubleSupport(const std::string calib_code, const int nr_of_samples );
    bool calibrateOffsetOnRightFootSingleSupport(const std::string calib_code, const int nr_of_samples );
    bool calibrateOffsetOnLeftFootSingleSupport(const std::string calib_code, const int nr_of_samples );
    bool resetOffset(const std::string calib_code);

    /**
     * Wait for the calibration to end and then return.
     *
     * @return always returns true
     */
    bool waitCalibrationDone();
    void run();
    void threadRelease();

    /**
     * For all the joints for which we are estimating torques,
     * set their interaction mode to INTERACTION_STIFF.
     * Furthermore, if their control mode is CM_TORQUE, switch
     *  it to CM_POSITION .
     *
     * \note this call is extremly expensive, given that it will
     *       create a lot of blocking RPC traffic. It should not be
     *       used in a loop, but just at specific isolated moments
     *       (module closing or module calibrating).
     */
    bool ensureJointsAreNotUsingTorqueEstimates();

    /**
     * Open the controlboardwrappers needed for the
     * ensureJointsAreNotUsingTorqueEstimates method.
     */
    bool openControlBoards();

    /**
     * Close the controlboardwrappers needed for the
     * ensureJointsAreNotUsingTorqueEstimates method.
     */
    bool closeControlBoards();


};


#endif
