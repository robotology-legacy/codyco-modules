#ifndef CODYCO_WHOLE_BODY_DYNAMICS_DEVICE_H
#define CODYCO_WHOLE_BODY_DYNAMICS_DEVICE_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RpcServer.h>
#include <yarp/dev/IVirtualAnalogSensor.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/GenericSensorInterfaces.h>

#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>

#include <wholeBodyDynamicsSettings.h>

#include <vector>


namespace yarp {
namespace dev {

struct virtualAnalogSensorRemappedAxis
{
    IVirtualAnalogSensor * dev;
    int localAxis;
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
 * Furthermore are also used to match the yarp axes to the joint names found in the passed URDF fie.
 */
class WholeBodyDynamicsDevice :  public yarp::dev::DeviceDriver,
                                 public yarp::dev::IMultipleWrapper,
                                 public yarp::os::RateThread
{
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
     * Flag set to false at the beginning, and to true only if the estimation
     * have been performed correctly.
     */
    bool estimationWentWell;

    /** Remapped controlboard containg the axes for which the joint torques are estimated */
    yarp::dev::PolyDriver remappedControlBoard;
    struct remappedControlBoardInterfacesStruct
    {
        yarp::dev::IEncoders        * encs;
        yarp::dev::IMultipleWrapper * multwrap;
    };

    remappedControlBoardInterfacesStruct remappedControlBoardInterfaces;

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
    yarp::os::RpcServer settingsPort;

    /**
     * Open-related methods
     */

    bool openSettings();
    bool openRemapperControlBoard(os::Searchable& config);
    bool openEstimator(os::Searchable& config);

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
    void computeExternalForcesAndJointTorques();

    // Publish related methods
    void publishTorques();
    void publishEstimatedQuantities();

    // Publish related attributes (port, interfaces)
    std::vector<virtualAnalogSensorRemappedAxis> analogSensorAxes;


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
    yarp::os::Vector               ftMeasurement;
    yarp::os::Vector               imuMeasurement;
    iDynTree::SensorsMeasurements  sensorsMeasurements;
    iDynTree::LinkUnknownWrenchContacts measuredContactLocations;
    iDynTree::JointDOFsDoubleArray estimatedJointTorques;
    iDynTree::LinkContactWrenches  estimateExternalContactWrenches;




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