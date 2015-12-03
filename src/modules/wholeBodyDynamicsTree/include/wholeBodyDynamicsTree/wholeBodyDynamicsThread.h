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
#include "wholeBodyDynamicsTree/wholeBodyDynamicsStatesInterface.h"
#include "wholeBodyDynamicsTree/simpleLeggedOdometry.h"

#include "ctrlLibRT/filters.h"
#include "wholeBodyDynamicsTree/robotStatus.h"

struct outputTorquePortInformation
{
    std::string port_name;
    int magic_number;
    std::vector< int > wbi_numeric_ids_to_publish;
    yarp::sig::Vector output_vector;
    yarp::os::BufferedPort<yarp::os::Bottle> * output_port;
};

struct outputWrenchPortInformation
{
    std::string port_name;
    std::string link;
    std::string orientation_frame;
    std::string origin_frame;
    int link_index;
    int orientation_frame_index;
    int origin_frame_index;
    yarp::sig::Vector output_vector;
    yarp::os::BufferedPort<yarp::sig::Vector> * output_port;
};

class wholeBodyDynamicsFilters
{
    public:

    wholeBodyDynamicsFilters(int nrOfDOFs,
                             int nrOfFTSensors,
                             double cutoffInHzIMU,
                             double periodInSeconds,
                             bool enableFTFiltering,
                             double cutoffInHzFT,
                             bool enableVelAccFiltering,
                             double cutoffInHzVelAcc);
    ~wholeBodyDynamicsFilters();

    iCub::ctrl::realTime::FirstOrderLowPassFilter * imuLinearAccelerationFilter; ///<  low pass filters for IMU linear accelerations
    iCub::ctrl::realTime::FirstOrderLowPassFilter * imuAngularVelocityFilter; ///< low pass filters for IMU angular velocity
    std::vector<iCub::ctrl::realTime::FirstOrderLowPassFilter *> forcetorqueFilters; ///< low pass filters for ForceTorque sensors
    iCub::ctrl::realTime::FirstOrderLowPassFilter * jointVelFilter; ///< low pass filters for joint velocities
    iCub::ctrl::realTime::FirstOrderLowPassFilter * jointAccFilter; ///< low pass filters for joint accelerations

    // Adaptive filter for imuAngularAcceleration estimation
    iCub::ctrl::AWLinEstimator * imuAngularAccelerationFilt;
    iCub::ctrl::AWPolyElement imuAngularAccelerationFiltElement;

    //Flag for checking if filtering of FT and joint velAcc is enabled or not
    bool enableFTFiltering;
    bool enableVelAccFiltering;
};

class OffsetSmoother
{
    public:
    double smooth_calibration_period_in_seconds;
    OffsetSmoother(int nrOfFTSensors, double smoothingTimeInSeconds);
    void reset(int nrOfFTSensors, double smoothingTimeInSeconds);

    std::vector<yarp::sig::Vector> old_offset;
    std::vector<yarp::sig::Vector> new_offset;
    std::vector<double>            initial_smoothing_time;
    std::vector<bool>              is_smoothing;

    void setNewOffset(double current_time, unsigned int ft_id, const yarp::sig::Vector & new_offset, const yarp::sig::Vector & old_offset);
    void updateOffset(const double current_time, const unsigned int ft_id, yarp::sig::Vector & smoothed_offset);

};

/**
 *
  */
class wholeBodyDynamicsThread: public yarp::os::RateThread
{
public:
     enum threadStatusEnum {STATUS_OK=0, STATUS_DISCONNECTED};
private:
    /**
     * Status of the thread (OK or disconnected)
     */
    threadStatusEnum threadStatus;

    /**
     * Counter of consecutive failed sensor readings.
     * After there have been 10 (hardcoded value for now)
     * consecutive failed attemps to read the sensors,
     * the module automatically closes.
     *
     * See https://github.com/robotology/codyco-modules/issues/161
     */
    int nrOfConsecutiveFailedSensorReadings;

    /** prefix for all the ports opened by this module */
    std::string moduleName;
    /** prefix for all the ports of the robot at which we are connecting */
    std::string robotName;

    /** wholeBodySensors interface to get sensors readings */
    wbi::iWholeBodySensors * sensors;

    /** helper class for estimating external wrenches and internal torques */
    ExternalWrenchesAndTorquesEstimator * externalWrenchTorqueEstimator;

    /** helper class for calibrating 6-axis F/T sensors offsets */
    // OffsetEstimator offsetEstimator;

    /** filter class */
    wholeBodyDynamicsFilters * filters;

    /** method to correctly configure the filters class */
    //initFilters();

    /** offset smoothe class */
    OffsetSmoother * offset_smoother;

    /** helper variable for printing every printPeriod milliseconds */
    int                 printCountdown;
    /** period after which some diagnostic messages are print */
    double              printPeriod;

    /** Mode of operation of the thread: normal operation or calibration */
    enum { NORMAL, CALIBRATING, CALIBRATING_ON_DOUBLE_SUPPORT } wbd_mode;

    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> *port_contacts_input;

    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> *port_contacts_output;

    yarp::os::BufferedPort<yarp::sig::Vector> * port_icubgui_base;

    yarp::os::BufferedPort<yarp::sig::Vector> * port_filtered_inertial;

    std::vector<yarp::os::BufferedPort<yarp::sig::Vector> *> port_filtered_ft;

    yarp::os::Stamp timestamp;

    void setNewFTOffset(const int ft_sensor_id, const yarp::sig::Vector & new_offset);

    template <class T> void broadcastData(T& _values, yarp::os::BufferedPort<T> *_port);
    void closePort(yarp::os::Contactable *_port);
    void writeTorque(const yarp::sig::Vector & _values, int _address, yarp::os::BufferedPort<yarp::os::Bottle> *_port);
    void publishTorques();
    void publishContacts();
    void getExternalWrenches();
    void publishExternalWrenches();
    void publishBaseToGui();
    void publishFilteredInertialForGravityCompensator();
    void publishFilteredFTWithoutOffset();
    bool decodeCalibCode(const std::string calib_code);
    void disableCalibration();


    wbi::ID convertFTiDynTreeToFTwbi(int ft_sensor_id);

    /**
     * Run the main loop when the module is actually performing estimation.
     * @param sensorReadSuccessfull flag passed to mark if the read of the sensor was successful.
     *                              If false, there was some error while reading the sensors.
     */
    void estimation_run(const bool sensorReadSuccessfull);
    void calibration_run();
    void calibration_on_double_support_run();


    //Data structures for mapping between wbi and output ports
    // this are populated by the WBD_TORQUE_PORTS group
    std::vector< outputTorquePortInformation > output_torque_ports;

    // Data structures for wrenches to publish on individual external wrenches
    // (the complete external force information is published in the contacts:o
    //  port, but for backward compatibility we have to stream external wrenches
    //  informations on individual ports
    std::vector< outputWrenchPortInformation > output_wrench_ports;

    bool loadExternalWrenchesPortsConfigurations();

    bool openExternalWrenchesPorts();
    bool closeExternalWrenchesPorts();

    bool loadEstimatedTorquesPortsConfigurations();

    iCub::skinDynLib::skinContactList external_forces_list;

    yarp::sig::Vector iCubGuiBase;
    yarp::sig::Vector FilteredInertialForGravityComp;

    //Calibration related variables
    yarp::sig::Vector zero_three_elem_vector;
    yarp::sig::Vector zero_dof_elem_vector;
    yarp::sig::Vector calibration_ddp;

    bool smooth_calibration;
    bool first_calibration;
    double smooth_calibration_period_in_ms;
    yarp::os::Mutex run_mutex;
    bool run_mutex_acquired;
    yarp::os::Mutex calibration_mutex;
    RobotJointStatus joint_status;
    RobotSensorStatus sensor_status;

    iCub::iDynTree::TorqueEstimationTree * icub_model_calibration;
    iCub::iDynTree::TorqueEstimationTree * icub_model_world_base_position;
    iCub::iDynTree::TorqueEstimationTree * icub_model_calibration_on_l_sole;
    iCub::iDynTree::TorqueEstimationTree * icub_model_calibration_on_r_sole;
    std::string calibration_support_link;

    int samples_requested_for_calibration;
    int max_samples_for_calibration;

    std::vector<int> arms_fts;
    std::vector<int> legs_fts;
    std::vector<int> feet_fts;

    //End Calibration related variables
    int left_foot_link_idyntree_id;
    int right_foot_link_idyntree_id;
    int root_link_idyntree_id;

    // iCubGui related variables
    int icubgui_support_frame_idyntree_id;
    KDL::Frame initial_world_H_supportFrame;


    yarp::sig::Matrix transform_mat_buffer;

    int samples_used_for_calibration;

    std::vector<bool> calibrate_ft_sensor;
    std::vector<yarp::sig::Vector> offset_buffer;
    //End of Calibration related variables

    bool assume_fixed_base_calibration;
    std::string fixed_link_calibration;
    bool assume_fixed_base_calibration_from_odometry;

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

    ///////////////////////////////////////////////////////////
    ///// simpleLeggedOdometry private attributes and methods
    ///////////////////////////////////////////////////////////
    simpleLeggedOdometry odometry_helper;
    int odometry_floating_base_frame_index;
    yarp::sig::Matrix world_H_floatingbase;
    yarp::sig::Vector floatingbase_twist;
    yarp::sig::Vector floatingbase_acctwist;
    bool odometry_enabled;
    yarp::os::BufferedPort<yarp::os::Bottle> * port_floatingbasestate;
    bool frames_streaming_enabled;
    yarp::os::BufferedPort<yarp::os::Property> * port_frames;
    std::vector<int> frames_to_stream_indices;
    std::vector<std::string> frames_to_stream;
    std::vector<yarp::os::Bottle> buffer_bottles;
    yarp::sig::Matrix buffer_transform_matrix;
    bool com_streaming_enabled;
    yarp::os::BufferedPort<yarp::sig::Vector> * port_com;
    std::string current_fixed_link_name;
    bool initOdometry();
    void publishOdometry();
    void closeOdometry();

public:
    wholeBodyDynamicsThread(std::string _name,
                            std::string _robotName,
                            int _period,
                            yarpWbi::yarpWholeBodySensors *_wbi,
                            yarp::os::Property & yarpOptions,
                            bool assume_fixed_base_calibration,
                            std::string fixed_link,
                            bool assume_fixed_base_calibration_from_odometry
                           );

    bool threadInit();

    // rpc methods
    bool calibrateOffset(const std::string calib_code, const int nr_of_samples );
    bool calibrateOffsetOnDoubleSupport(const std::string calib_code, const int nr_of_samples );
    bool calibrateOffsetOnRightFootSingleSupport(const std::string calib_code, const int nr_of_samples );
    bool calibrateOffsetOnLeftFootSingleSupport(const std::string calib_code, const int nr_of_samples );
    bool resetOffset(const std::string calib_code);

    // rpc simple odometry methods
    virtual bool resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link);
    virtual bool changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link);

    /**
     * Wait for the calibration to end and then return.
     *
     * @return always returns true
     */
    bool waitCalibrationDone();

    /**
     * Read the robot sensors from the yarpWholeBodySensors interface.
     *
     * @return true if all went well, false if errors occured while reading sensors.
     */
    bool readRobotStatus();

    void run();
    void threadRelease();

    /**
     * Get the thread status (STATUS_OK or STATUS_DISCONNECTED).
     * This method calls the same mutex locked in the run() method
     * of the thread, so don't call it from the run() if you don't
     * want to reach a deadlock.
     *
     * @return the status of the thread
     */
    threadStatusEnum getThreadStatus();

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
