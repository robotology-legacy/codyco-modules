#ifndef CODYCO_FLOATING_BASE_ESTIMATOR_H
#define CODYCO_FLOATING_BASE_ESTIMATOR_H

// YARP includes
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/RateThread.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>

// iDynTree includes
#include <iDynTree/Estimation/SimpleLeggedOdometry.h>

#include <floatingBaseEstimator_IDLServer.h>


#include <vector>


namespace yarp {
namespace dev {

/**
 * \section floatingBaseEstimator
 * A device that takes a list of axes and estimates the .
 *
 *  The parameters taken in input by this device are:
 * | Parameter name | SubParameter   | Type              | Units | Default Value | Required |   Description                                                     | Notes |
 * |:--------------:|:--------------:|:-----------------:|:-----:|:-------------:|:--------:|:-----------------------------------------------------------------:|:-----:|
 * | axesNames      |      -         | vector of strings |   -   |   -           | Yes      | Ordered list of the axes that are part of the remapped device.    |       |
 * | modelFile      |      -         | path to file      |   -   | model.urdf    | No       | Path to the URDF file used for the kinematic and dynamic model.   |       |
 * | initialFixedFrame  | string | - | - | Yes | Name of a frame attached to the link that is assumed to be fixed at start | - |
 * | initialWorldFrame | string | - | Equal to initialFixedFrame | No | Name of the frame of the model that is supposed to be coincident with the world/inertial frame at start | - |
 *
 * The axes contained in the axesNames parameter are then mapped to the wrapped controlboard in the attachAll method, using controlBoardRemapper class.
 * Furthermore are also used to match the yarp axes to the joint names found in the passed URDF file.
 *
 *
 * \subsection ConfigurationExamples
 *
 * Example onfiguration file using .ini format.
 *
 * \code{.unparsed}
 *  device floatingbaseestimator
 *  axesNames (joint1 joint2 joint3)
 *
 * ...
 * \endcode
 *
 * Example configuration file using .xml yarprobotinterface format.
 * \code{.xml}
 * \endcode
 *
 */
class floatingBaseEstimator :  public yarp::dev::DeviceDriver,
                               public yarp::dev::IMultipleWrapper,
                               public yarp::os::RateThread,
                               public floatingBaseEstimator_IDLServer
{
private:
    /**
     * Port prefix used for all the ports opened by wholeBodyDynamics.
     */
    std::string portPrefix;

    /**
     * Flag set to false at the beginning, and to true only when sensors have been read correctly.
     */
    bool sensorReadCorrectly;

    /**
     * Flag set to false at the beginning, and to true only if attachAll has been correctly called
     * and at least one run has been correctly completed.
     */
    bool correctlyConfigured;

    /**
     * Flag set to false at the beginning, and to true only if the estimation
     * have been performed correctly.
     */
    bool estimationWentWell;

    /**
     * Names of the axis (joint with at least a degree of freedom) used in estimation.
     */
    std::vector<std::string> estimationJointNames;

    /** Remapped controlboard containg the axes used for estimating the floating base position. */
    yarp::dev::PolyDriver remappedControlBoard;
    struct
    {
        yarp::dev::IEncoders        * encs;
        yarp::dev::IMultipleWrapper * multwrap;
    } remappedControlBoardInterfaces;


    /**
     * Mutex to protect the settings data structure, and all the data in
     * the class that is accessed by the run method, the attachAll methods
     * (managed by the yarprobotinterface thread) and by the RPC call
     * invoked by the RPC thread.
     */
    yarp::os::Mutex deviceMutex;

    /**
     * Open-related methods
     */
    bool openPorts();
    bool openRemapperControlBoard(os::Searchable& config);
    bool openEstimator(os::Searchable& config);

    /**
     * Close-related methods
     */
    bool closePorts();

    /**
     * Attach-related methods
     */

    /**
     * Attach all controlboard devices.
     * A device is identified as a controlboard if it
     * implements the IEncoders interface.
     */
    bool attachAllControlBoard(const PolyDriverList& p);

    /**
     * Run-related methods.
     */

    /**
     * Return true if we were able to read the sensors and update
     * the internal buffers, false otherwise.
     */
    void readSensors();
    void updateKinematics();

    // Publish related methods
    void publishEstimatedQuantities();
    void publishFloatingBasePosInWBIFormat();
    void publishFloatingBasePosIniCubGuiFormat();

    /**
     * Load settings from config.
     */
    bool loadSettingsFromConfig(yarp::os::Searchable& config);

    /**
     * Class actually doing computations.
     */
    iDynTree::SimpleLeggedOdometry estimator;

    /**
     * Buffers related methods
     */
    void resizeBuffers();

    /*
     * Buffers
     *
     */

    /// < Joint position read from controlboard
    iDynTree::JointPosDoubleArray  jointPos;

    /// < Joint velocities (for now set to zero)
    iDynTree::JointDOFsDoubleArray jointVelSetToZero;

    /**
     * RPC Calibration related attributes
     */
    yarp::os::Port  rpcPort;        // a port to handle rpc messages

    /**
     * Port for publishing data consumed by the WBI
     */
    yarp::os::BufferedPort<yarp::os::Bottle> WBIPort;

    /**
     * Port for publishing data consumed by the iCubGui
     */
    yarp::os::BufferedPort<yarp::sig::Vector> iCubGuiPort;

    // Buffers
    yarp::sig::Matrix homMatrixBuffer;

    // Settings
    std::string initialWorldFrame;
    std::string initialFixedFrame;


public:
    // CONSTRUCTOR
    floatingBaseEstimator();
    ~floatingBaseEstimator();

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

#endif /* CODYCO_FLOATING_BASE_ESTIMATOR_H */

