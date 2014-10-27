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

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <wbi/wbi.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarpWholeBodyInterface/yarpWholeBodyStatesLocal.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;
using namespace std;
using namespace wbi;
using namespace Eigen;


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

/**
 *
  */
class wholeBodyDynamicsThread: public RateThread
{
    string              name;
    string              robotName;
    yarpWbi::yarpWholeBodyStatesLocal *estimator;

    int                 printCountdown;         // every time this is 0 (i.e. every PRINT_PERIOD ms) print stuff
    double              PRINT_PERIOD;

    enum { NORMAL, CALIBRATING, CALIBRATING_ON_DOUBLE_SUPPORT } wbd_mode;     /// < Mode of operation of the thread: normal operation or calibration

    //output ports
    ///< \todo TODO add a proper structure for output ports, by dividing them for body parts or sensors
    BufferedPort<Bottle> *port_RATorques;
    BufferedPort<Bottle> *port_RLTorques;
    BufferedPort<Bottle> *port_RWTorques;
    BufferedPort<Bottle> *port_LATorques;
    BufferedPort<Bottle> *port_LLTorques;
    BufferedPort<Bottle> *port_LWTorques;
    BufferedPort<Bottle> *port_TOTorques;
    BufferedPort<Bottle> *port_HDTorques;

    BufferedPort<Vector> *port_external_wrench_RA;
    BufferedPort<Vector> *port_external_wrench_LA;
    BufferedPort<Vector> *port_external_wrench_RL;
    BufferedPort<Vector> *port_external_wrench_LL;

    BufferedPort<Vector> *port_external_cartesian_wrench_RA;
    BufferedPort<Vector> *port_external_cartesian_wrench_LA;
    BufferedPort<Vector> *port_external_cartesian_wrench_RL;
    BufferedPort<Vector> *port_external_cartesian_wrench_LL;

    BufferedPort<Vector> *port_sensor_wrench_RL;
    BufferedPort<Vector> *port_sensor_wrench_LL;
    BufferedPort<Vector> *port_model_wrench_RL;
    BufferedPort<Vector> *port_model_wrench_LL;


    BufferedPort<iCub::skinDynLib::skinContactList> *port_contacts;

    /*
    BufferedPort<Vector> *port_all_accelerations;
    BufferedPort<Vector> *port_all_velocities;
    BufferedPort<Vector> *port_all_positions;
    */

    // ports outputing the external dynamics seen at the F/T sensor
    /*
    BufferedPort<Vector> *port_external_ft_arm_left;
    BufferedPort<Vector> *port_external_ft_arm_right;
    BufferedPort<Vector> *port_external_ft_leg_left;
    BufferedPort<Vector> *port_external_ft_leg_right;
    */

    BufferedPort<Vector> * port_icubgui_base;


    yarp::os::Stamp timestamp;

    template <class T> void broadcastData(T& _values, BufferedPort<T> *_port);
    void closePort(Contactable *_port);
    void writeTorque(Vector _values, int _address, BufferedPort<Bottle> *_port);
    void publishTorques();
    void publishContacts();
    void getEndEffectorWrenches();
    void publishEndEffectorWrench();
    void publishBaseToGui();
    void publishAnkleFootForceTorques();

    wbi::wbiId convertFTiDynTreeToFTwbi(int ft_sensor_id);
    void normal_run();
    void calibration_run();
    void calibration_on_double_support_run();


    //Buffer vectors
    yarp::sig::Vector all_torques;
    yarp::sig::Vector TOTorques;
    yarp::sig::Vector HDTorques;
    yarp::sig::Vector LATorques;
    yarp::sig::Vector RATorques;
    yarp::sig::Vector LLTorques;
    yarp::sig::Vector RLTorques;

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

    //Calibration related variables
    yarp::os::Mutex run_mutex;
    yarp::os::Mutex calibration_mutex;
    iCubTreeStatus tree_status;

    iCub::iDynTree::iCubTree_version_tag icub_version;
    iCub::iDynTree::iCubTree * icub_model_calibration;

    int samples_requested_for_calibration;
    int max_samples_for_calibration;
    int l_foot_ft_sensor_id;
    int r_foot_ft_sensor_id;

    int l_arm_ft_sensor_id;
    int r_arm_ft_sensor_id;

    int l_leg_ft_sensor_id;
    int r_leg_ft_sensor_id;

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
    BufferedPort<Vector> * port_joint_ankle_cartesian_wrench;
    BufferedPort<Vector> * port_joint_ankle_cartesian_wrench_from_model;
    BufferedPort<Vector> * port_joint_foot_cartesian_wrench;
    BufferedPort<Vector> * port_joint_foot_cartesian_wrench_from_model;
    iCub::iDynTree::iCubTree * icub_model_zmp;



public:

    wholeBodyDynamicsThread(string _name,
                            string _robotName,
                            int _period,
                            yarpWbi::yarpWholeBodyStatesLocal *_wbi,
                            const iCub::iDynTree::iCubTree_version_tag icub_version,
                            bool autoconnect,
                            bool assume_fixed_base_calibration,
                            std::string fixed_link,
                            bool zmp_test_mode, std::string foot_to_test);

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

};


#endif
