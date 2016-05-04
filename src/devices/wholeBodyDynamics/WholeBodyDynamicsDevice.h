#ifndef CODYCO_WHOLE_BODY_DYNAMICS_DEVICE_H
#define CODYCO_WHOLE_BODY_DYNAMICS_DEVICE_H

// YARP includes
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IVirtualAnalogSensor.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/GenericSensorInterfaces.h>

// iCub includes
#include <iCub/skinDynLib/skinContactList.h>

// iDynTree includes
#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>
#include <iDynTree/iCub/skinDynLibConversions.h>

// Filters
#include "ctrlLibRT/filters.h"

#include <wholeBodyDynamicsSettings.h>
#include <wholeBodyDynamics_IDLServer.h>

#include <vector>


namespace yarp {
namespace dev {

struct virtualAnalogSensorRemappedAxis
{
    IVirtualAnalogSensor * dev;
    int localAxis;
};

class wholeBodyDynamicsDeviceFilters
{
    public:

    wholeBodyDynamicsDeviceFilters();

    /**
     * Allocate the filters.
     */
    void init(int nrOfFTSensors,
         double initialCutOffForFTInHz,
         double initialCutOffForIMUInHz,
         double periodInSeconds);

    void updateCutOffFrequency(double initialCutOffForFTInHz,
                               double initialCutOffForIMUInHz);

    /**
     * Deallocate the filters
     */
    void fini();


    ~wholeBodyDynamicsDeviceFilters();

    ///<  low pass filters for IMU linear accelerations
    iCub::ctrl::realTime::FirstOrderLowPassFilter * imuLinearAccelerationFilter;
    ///< low pass filters for IMU angular velocity
    iCub::ctrl::realTime::FirstOrderLowPassFilter * imuAngularVelocityFilter;
    ///< low pass filters for ForceTorque sensors
    std::vector<iCub::ctrl::realTime::FirstOrderLowPassFilter *> forcetorqueFilters;
};

/**
 * \section WholeBodyDynamicsDevice
 * A device that takes a list of axes and estimates the joint torques for each one of this axes.
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                    | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:--------------------------: |:-----------------------------------------------------------------:|:-----:|
 * | axesNames      |      -         | vector of strings  | -      |   -           | Yes     | Ordered list of the axes that are part of the remapped device. |  |
 * | modelFile      |      -         | path to file |    -      | model.urdf    | No                          | Path to the URDF file used for the kinematic and dynamic model. |
 *
 * The axes are then mapped to the wrapped controlboard in the attachAll method, using controlBoardRemapper class.
 * Furthermore are also used to match the yarp axes to the joint names found in the passed URDF file.
 *
 *
 * Configuration file using .ini format.
 *
 * \code{.unparsed}
 *  device controlboardremapper
 *  axesNames (joint1 joint2 joint3)
 *
 * ...
 * \endcode
 *
 *
 */
class WholeBodyDynamicsDevice :  public yarp::dev::DeviceDriver,
                                 public yarp::dev::IMultipleWrapper,
                                 public yarp::os::RateThread,
                                 public wholeBodyDynamics_IDLServer
{
    struct imuMeasurements
    {
        iDynTree::Vector3 linProperAcc;
        iDynTree::Vector3 angularVel;
        iDynTree::Vector3 angularAcc;
    };

private:
    /**
     * Port prefix used for all the ports opened by wholeBodyDynamics.
     */
    std::string portPrefix;

    /**
     * Flag set to false at the beginning, and to true only if attachAll has been correctly called.
     */
    bool correctlyConfigured;

    /**
     *
     *
     */
    bool sensorReadCorrectly;

    /**
     * Flag set to false at the beginning, and to true only if the estimation
     * have been performed correctly.
     */
    bool estimationWentWell;

    /**
     * Flag set to false initially, then to true as soon as a valid offset is available
     * (so torque estimation can start to be broadcasted).
     */
    bool validOffsetAvailable;


    /**
     * Names of the axis (joint with at least a degree of freedom) used in estimation.
     */
    std::vector<std::string> estimationJointNames;

    /** Remapped controlboard containg the axes for which the joint torques are estimated */
    yarp::dev::PolyDriver remappedControlBoard;
    struct
    {
        yarp::dev::IEncoders        * encs;
        yarp::dev::IMultipleWrapper * multwrap;
    } remappedControlBoardInterfaces;


    /** F/T sensors interfaces */
    std::vector<yarp::dev::IAnalogSensor * > ftSensors;

    /** IMU interface */
    yarp::dev::IGenericSensor * imuInterface;

    /**
     * Vector containg the information about the IVirtualAnalogSensor devices
     * on which to publish the estimated torques
     */
    std::vector<virtualAnalogSensorRemappedAxis> remappedVirtualAnalogSensorAxis;

    /**
     * Setting for the whole body external wrenches and joint torques estimation.
     * Contained in a Thrift-generated structure to enable easy editing through
     * a YARP RPC port.
     */
    wholeBodyDynamicsSettings settings;
    wholeBodyDynamicsSettings::Editor settingsEditor;

    /**
     * Mutex to protect the settings data structure, and all the data in
     * the class that is accessed by the run method, the attachAll methods
     * (managed by the yarprobotinterface thread) and by the RPC call
     * invoked by the RPC thread.
     */
    yarp::os::Mutex deviceMutex;

    /**
     * A port for editing remotly the setting of wholeBodyDynamics
     */
    yarp::os::Port settingsPort;

    /**
     * Open-related methods
     */

    bool openSettingsPort();
    bool openRPCPort();
    bool openRemapperControlBoard(os::Searchable& config);
    bool openEstimator(os::Searchable& config);
    bool openDefaultContactFrames(os::Searchable& config);
    bool openSkinContactListPorts(os::Searchable& config);

    /**
     * Close-related methods
     */
    bool closeSettingsPort();
    bool closeRPCPort();

    /**
     * Attach-related methods
     */
    bool attachAllControlBoard(const PolyDriverList& p);
    bool attachAllVirtualAnalogSensor(const PolyDriverList& p);
    bool attachAllFTs(const PolyDriverList& p);
    bool attachAllIMUs(const PolyDriverList& p);

    /**
     * Run-related methods.
     */
    void readSensors();
    void filterSensorsAndRemoveSensorOffsets();
    void updateKinematics();
    void readContactPoints();
    void computeCalibration();
    void computeExternalForcesAndJointTorques();



    // Publish related methods
    void publishTorques();
    void publishContacts();
    void publishEstimatedQuantities();

    /**
     * Load settings from config.
     */
    bool loadSettingsFromConfig(yarp::os::Searchable& config);

    /**
     * Class actually doing computations.
     */
    iDynTree::ExtWrenchesAndJointTorquesEstimator estimator;

    /**
     * Buffers related methods
     */
    void resizeBuffers();

    /*
     * Buffers
     *
     */
    iDynTree::JointPosDoubleArray  jointPos;
    iDynTree::JointDOFsDoubleArray jointVel;
    iDynTree::JointDOFsDoubleArray jointAcc;
    yarp::sig::Vector              ftMeasurement;
    yarp::sig::Vector              imuMeasurement;
    /***
     * Buffer for raw sensors measurements.
     */
    iDynTree::SensorsMeasurements  rawSensorsMeasurements;
    imuMeasurements                rawIMUMeasurements;

    /**
     * Filters
     */
    wholeBodyDynamicsDeviceFilters filters;

    /**
     * Buffer for filtered (both to reduce noise and remove offset) sensors.
     */
    iDynTree::SensorsMeasurements  filteredSensorMeasurements;
    imuMeasurements                filteredIMUMeasurements;

    iDynTree::LinkUnknownWrenchContacts measuredContactLocations;
    iDynTree::JointDOFsDoubleArray estimatedJointTorques;
    iDynTree::LinkContactWrenches  estimateExternalContactWrenches;

    /**
     * Calibration buffers
     */
    struct
    {
        bool ongoingCalibration;
        std::vector<bool> calibratingFTsensor;
        std::vector<iDynTree::Vector6> offsetSumBuffer;
        std::vector<iDynTree::Wrench>  offset;
        iDynTree::LinkUnknownWrenchContacts assumedContactLocationsForCalibration;
        iDynTree::SensorsMeasurements  predictedSensorMeasurementsForCalibration;
        iDynTree::JointDOFsDoubleArray predictedJointTorquesForCalibration;
        iDynTree::LinkContactWrenches  predictedExternalContactWrenchesForCalibration;
        size_t nrOfSamplesUsedUntilNowForCalibration;
        size_t nrOfSamplesToUseForCalibration;
    } calibrationBuffers;

    /**
      * Semaphore used by the RPC to wait until the calibration is complete.
      */
    yarp::os::Semaphore calibrationSemaphore;

    /***
     * RPC Calibration related methods
     */
    /**
      * Calibrate the force/torque sensors
      * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the torso/waist)
      * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
      * @param nr_of_samples number of samples
      * @return true/false on success/failure
      */
     virtual bool calib(const std::string& calib_code, const int32_t nr_of_samples = 100);

     /**
      * Calibrate the force/torque sensors when on double support
      * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the sole).
      * For this calibration the strong assumption of simmetry of the robot and its pose is done.
      * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
      * @param nr_of_samples number of samples
      * @return true/false on success/failure
      */
     virtual bool calibStanding(const std::string& calib_code, const int32_t nr_of_samples = 100);

     /**
      * Calibrate the force/torque sensors when on single support on left foot
      * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the left sole).
      * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
      * @param nr_of_samples number of samples
      * @return true/false on success/failure
      */
     virtual bool calibStandingLeftFoot(const std::string& calib_code, const int32_t nr_of_samples = 100);

     /**
      * Calibrate the force/torque sensors when on single support on right foot
      * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the right sole).
      * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
      * @param nr_of_samples number of samples
      * @return true/false on success/failure
      */
     virtual bool calibStandingRightFoot(const std::string& calib_code, const int32_t nr_of_samples = 100);

     /**
      * Reset the sensor offset to 0 0 0 0 0 0 (six zeros).
      * @param calib_code argument to specify the sensors to reset (all,arms,legs,feet)
      * @return true/false on success/failure
      */
     virtual bool resetOffset(const std::string& calib_code);

     /**
      * Quit the module.
      * @return true/false on success/failure
      */
     virtual bool quit();

     /**
      * Reset the odometry world to be (initially) a frame specified in the robot model,
      * and specify a link that is assumed to be fixed in the odometry.
      * @param initial_world_frame the frame of the robot model that is assume to be initially
      *        coincident with the world/inertial frame.
      * @param new_fixed_link the name of the link that should be initially fixed
      * @return true/false on success/failure (typically if the frame/link names are wrong)
      */
     virtual bool resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link);

     /**
      * Change the link that is considered fixed by the odometry.
      * @param new_fixed_link the name of the new link that should be considered fixed
      * @return true/false on success/failure (typically if the frame/link names are wrong)
      */
     virtual bool changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link);
     
     /**
      * Set the cutoff frequency (in Hz) for IMU measurements
      * @return true/false on success/failure
      */
     virtual bool set_imuFilterCutoffInHz(const double newCutoff);
      /**
      * Get the cutoff frequency (in Hz) for IMU measurements
      * @return the cutoff frequency (in Hz)
      */
      virtual double get_imuFilterCutoffInHz();
      /**
      * Set the cutoff frequency (in Hz) for FT measurements
      * @return true/false on success/failure
      */
      virtual bool set_forceTorqueFilterCutoffInHz(const double newCutoff);
      /**
      * Get the cutoff frequency (in Hz) for FT measurements
      * @return the cutoff frequency (in Hz)
      */
    virtual double get_forceTorqueFilterCutoffInHz();

     void setupCalibrationCommonPart(const int32_t nrOfSamples);
     bool setupCalibrationWithExternalWrenchOnOneFrame(const std::string & frameName, const int32_t nrOfSamples);
     bool setupCalibrationWithExternalWrenchesOnTwoFrames(const std::string & frame1Name, const std::string & frame2Name, const int32_t nrOfSamples);


     /**
      * RPC Calibration related attributes
      */
     yarp::os::Port  rpcPort;        // a port to handle rpc messages

     /**
      * Generic helper methods
      */
     size_t getNrOfFTSensors();
     void waitEndOfCalibration();
     void endCalibration();

     /**
      * List of frames in which a contact is assumed to be occuring
      * if no information about contacts is coming from the skin.
      * The order of the frames act as a priority:
      * the list is scanned if for a given subtree no contact is
      * specified, and a full wrench on the first suitable frame (i.e. frame belonging to the submodel)
      *  is added.
      */
     std::vector<std::string> defaultContactFrames;
     std::vector<iDynTree::FrameIndex> subModelIndex2DefaultContact;

     /**
      * Port used to read the location of external contacts
      * obtained from the skin.
      */
     yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> portContactsInput;

     /**
      * \todo we should use directly the class in the BufferedPort portContactsInput
      */
     iCub::skinDynLib::skinContactList contactsReadFromSkin;

     /**
      * Port used to publish the external forces acting on the
      * robot.
      */
     yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> portContactsOutput;

     /**
      * \todo we should use directly the class in the BufferedPort portContactsOutput
      */
     iCub::skinDynLib::skinContactList contactsEstimated;

     /**
      * Helper to convert between iDynTree and skinDynLib related datastructures.
      */
     iDynTree::skinDynLibConversionsHelper conversionHelper;

public:
    // CONSTRUCTOR
    WholeBodyDynamicsDevice();
    ~WholeBodyDynamicsDevice();

    // DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    // IMULTIPLE WRAPPER
    virtual bool attachAll(const yarp::dev::PolyDriverList &p);
    virtual bool detachAll();

    // RATE THREAD
    virtual void run();
};

}
}

#endif /* CODYCO_WHOLE_BODY_DYNAMICS_DEVICE_H */