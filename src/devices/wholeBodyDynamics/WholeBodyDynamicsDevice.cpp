#include "WholeBodyDynamicsDevice.h"

#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/GenericSensorInterfaces.h>

#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Core/Utils.h>

#include <cassert>
#include <cmath>

namespace yarp
{
namespace dev
{

WholeBodyDynamicsDevice::WholeBodyDynamicsDevice(): RateThread(10),
                                                    portPrefix("/wholeBodyDynamics"),
                                                    correctlyConfigured(false),
                                                    sensorReadCorrectly(false),
                                                    estimationWentWell(false),
                                                    imuInterface(0),
                                                    ongoingCalibration(false),
                                                    settingsEditor(settings),
                                                    calibrationSemaphore()
{
}

WholeBodyDynamicsDevice::~WholeBodyDynamicsDevice()
{

}


bool WholeBodyDynamicsDevice::openSettingsPort()
{
    settingsPort.setReader(settingsEditor);

    bool ok = settingsPort.open(portPrefix+"/settings");

    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice: Impossible to open port " << portPrefix+"/settings";
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::openRPCPort()
{
    this->wholeBodyDynamics_IDLServer::yarp().attachAsServer(rpcPort);

    bool ok = rpcPort.open(portPrefix+"/rpc");

    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice: Impossible to open port " << portPrefix+"/rpc";
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::closeSettingsPort()
{
    settingsPort.close();
    return true;
}

bool WholeBodyDynamicsDevice::closeRPCPort()
{
    rpcPort.close();
    return true;
}


bool WholeBodyDynamicsDevice::openRemapperControlBoard(os::Searchable& config)
{
    bool ok = remappedControlBoard.open(config);

    if( !ok )
    {
        return ok;
    }

    // View relevant interfaces for the remappedControlBoard
    ok = ok && remappedControlBoard.view(remappedControlBoardInterfaces.encs);
    ok = ok && remappedControlBoard.view(remappedControlBoardInterfaces.multwrap);

    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice::open impossible to use the necessary interfaces in remappedControlBoard";
        return ok;
    }

    // Check if the controlboard and the estimator have a consistent number of joints
    int axes = 0;
    remappedControlBoardInterfaces.encs->getAxes(&axes);
    if( axes != (int) estimator.model().getNrOfDOFs() )
    {
        yError() << "WholeBodyDynamicsDevice::open estimator model and the remappedControlBoard has an inconsistent number of joints";
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::openEstimator(os::Searchable& config)
{
    yarp::os::ResourceFinder & rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    std::string modelFileName = "model.urdf";
    if( config.check("modelFile") && config.find("modelFile").isString() )
    {
        modelFileName = config.find("modelFile").asString();
    }

    std::string modelFileFullPath = rf.findFileByName(modelFileName);
    bool ok = estimator.loadModelAndSensorsFromFile(modelFileFullPath);
    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice::open impossible to create ExtWrenchesAndJointTorquesEstimator from file "
                 << modelFileFullPath;
        return false;
    }

    this->resizeBuffers();
    return true;
}

bool WholeBodyDynamicsDevice::openDefaultContactFrames(os::Searchable& config)
{
    // For now we hardcode this info
    defaultContactFrames.push_back("l_hand");
    defaultContactFrames.push_back("r_hand");
    defaultContactFrames.push_back("root_link");
    defaultContactFrames.push_back("l_upper_leg");
    defaultContactFrames.push_back("r_upper_leg");
    defaultContactFrames.push_back("l_sole");
    defaultContactFrames.push_back("r_sole");

    // We build the defaultContactFramesIdx vector
    std::vector<iDynTree::FrameIndex> defaultContactFramesIdx;
    defaultContactFramesIdx.clear();
    for(size_t i=0; i < defaultContactFrames.size(); i++)
    {
        iDynTree::FrameIndex idx = estimator.model().getFrameIndex(defaultContactFrames[i]);

        if( idx == iDynTree::FRAME_INVALID_INDEX )
        {
            yWarning() << "Frame " << defaultContactFrames[i] << " not found in the model, discarding it" << std::endl;
        }
        else
        {
            defaultContactFramesIdx.push_back(idx);
        }
    }

    // For each submodel, we find the first suitable contact frame
    // This is a n^2 algorithm, but given that is just used in
    // configuration it should be ok
    size_t nrOfSubModels = estimator.submodels().getNrOfSubModels();

    for(int subModel=0; subModel < nrOfSubModels; subModel++)
    {
        for(int i=0; i < defaultContactFramesIdx.size(); i++)
        {
            estimator.submodels()
        }
    }



    return true;
}


void WholeBodyDynamicsDevice::resizeBuffers()
{
    this->jointPos.resize(estimator.model());
    this->jointVel.resize(estimator.model());
    this->jointAcc.resize(estimator.model());
    this->measuredContactLocations.resize(estimator.model());
    this->ftMeasurement.resize(6);
    this->imuMeasurement.resize(12);
    this->sensorsMeasurements.resize(estimator.sensors());
    this->estimatedJointTorques.resize(estimator.model());
    this->estimateExternalContactWrenches.resize(estimator.model());

    // Resize F/T stuff
    size_t nrOfFTSensors = estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);
    calibratingFTsensor.resize(nrOfFTSensors,false);
    iDynTree::Wrench zeroWrench;
    zeroWrench.zero();
    offset.resize(nrOfFTSensors,zeroWrench);
    offsetSumBuffer.resize(nrOfFTSensors,zeroWrench.asVector());
}


bool WholeBodyDynamicsDevice::loadSettingsFromConfig(os::Searchable& config)
{

}

bool WholeBodyDynamicsDevice::open(os::Searchable& config)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    bool ok;

    // Create the estimator
    ok = this->openEstimator(config);
    if( !ok ) return false;

    // Load settings in the class and in the estimator
    ok = this->loadSettingsFromConfig(config);
    if( !ok ) return false;

    // Open settings port
    ok = this->openSettingsPort();
    if( !ok ) return false;

    // Open the controlboard remapper
    ok = this->openRemapperControlBoard(config);
    if( !ok ) return false;

    ok = this->openDefaultContactFrames(config);
    if( !ok ) return false;



    return true;
}

bool WholeBodyDynamicsDevice::attachAllControlBoard(const PolyDriverList& p)
{
    PolyDriverList controlBoardList;
    for(size_t devIdx = 0; devIdx < p.size(); devIdx++)
    {
        IEncoders * pEncs = 0;
        if( p[devIdx]->poly->view(pEncs) )
        {
            controlBoardList.push(const_cast<PolyDriverDescriptor&>(*p[devIdx]));
        }
    }

    // Attach the controlBoardList to the controlBoardRemapper
    bool ok = remappedControlBoardInterfaces.multwrap->attachAll(controlBoardList);

    if( !ok )
    {
        yError() << " WholeBodyDynamicsDevice::attachAll in attachAll of the remappedControlBoard";
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::attachAllVirtualAnalogSensor(const PolyDriverList& p)
{
    // For the moment we assume that the controlboard device is the
    // same that implements the IVirtualAnalogSensor interface (this is how things
    // are implemented in embObjMotionControl) In general we at least assume that
    // all the devices that implement IVirtualAnalogSensor also implement IAxisInfo
    // to provide a name for the virtual analog axis
    std::vector<IVirtualAnalogSensor *> virtualAnalogList;
    std::vector<IAxisInfo *>            axisInfoList;
    for(size_t devIdx = 0; devIdx < p.size(); devIdx++)
    {
        IVirtualAnalogSensor * pVirtualAnalogSens = 0;
        IAxisInfo            * pAxisInfo          = 0;
        if( p[devIdx]->poly->view(pVirtualAnalogSens) )
        {
            virtualAnalogList.push_back(pVirtualAnalogSens);
            if( !(p[devIdx]->poly->view(pAxisInfo)) )
            {
                yError() << "WholeBodyDynamicsDevice::attachAll error: device "
                         << p[devIdx]->key << " exposes a IVirtualAnalogSensor, but not a IAxisInfo interface,"
                         << " impossible to map the list of joint names to the IVirtualAnalogSensor interface";
                return false;

            }
            else
            {
                axisInfoList.push_back(pAxisInfo);
            }
        }
    }

    // Find the axisName ---> device , localAxis mapping
    std::map<std::string, IVirtualAnalogSensor *> axisName2virtualAnalogSensorPtr;
    std::map<std::string, int> axisName2localAxis;

    for(size_t devIdx = 0; devIdx < virtualAnalogList.size(); devIdx++)
    {
        int nrOfVirtualAxes = virtualAnalogList[devIdx]->getChannels();
        for(int localAxis=0; localAxis < nrOfVirtualAxes; localAxis++)
        {
            yarp::os::ConstString axisName;
            axisInfoList[devIdx]->getAxisName(localAxis,axisName);

            std::string axisNameStd = axisName.c_str();

            axisName2virtualAnalogSensorPtr[axisNameStd] = virtualAnalogList[devIdx];
            axisName2localAxis[axisNameStd] = localAxis;
        }
    }

    // Save the axis ---> device, localAxis mapping
    remappedVirtualAnalogSensorAxis.resize(estimationJointNames.size());

    for(size_t axis = 0; axis < remappedVirtualAnalogSensorAxis.size(); axis++)
    {
        std::string jointName = estimationJointNames[axis];
        // if the name is not exposed in the VirtualAnalogSensors, just save a null pointer
        // and don't publish the torque estimated for that joint
        if( axisName2virtualAnalogSensorPtr.find(jointName) == axisName2virtualAnalogSensorPtr.end() )
        {
            remappedVirtualAnalogSensorAxis[axis].dev = 0;
            remappedVirtualAnalogSensorAxis[axis].localAxis = 0;
        }
        else
        {
            remappedVirtualAnalogSensorAxis[axis].dev = axisName2virtualAnalogSensorPtr[jointName];
            remappedVirtualAnalogSensorAxis[axis].localAxis = axisName2localAxis[jointName];
        }
    }

    return true;
}

bool WholeBodyDynamicsDevice::attachAllFTs(const PolyDriverList& p)
{
    std::vector<IAnalogSensor *> ftList;
    for(size_t devIdx = 0; devIdx < p.size(); devIdx++)
    {
        IAnalogSensor * pAnalogSens = 0;
        if( p[devIdx]->poly->view(pAnalogSens) )
        {
            ftList.push_back(pAnalogSens);
        }
    }

    if( ftList.size() != estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) )
    {
        yError() << " WholeBodyDynamicsDevice was expecting "
                 << estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE)
                 << " from the model, but got " << ftList.size() << " FT sensor in the attach list.";
        return false;
    }

    ftSensors.resize(ftList.size());
    for(int i=0; i < ftList.size(); i++)
    {
        ftSensors[i] = ftList[i];
    }

    return true;
}


bool WholeBodyDynamicsDevice::attachAllIMUs(const PolyDriverList& p)
{
    std::vector<IGenericSensor*> imuList;

    for(size_t devIdx = 0; devIdx < p.size(); devIdx++)
    {
        IGenericSensor * pGenericSensor = 0;
        if( p[devIdx]->poly->view(pGenericSensor) )
        {
            imuList.push_back(pGenericSensor);
        }
    }

    if( imuList.size() != 1 )
    {
        yError() << " WholeBodyDynamicsDevice::attachAll only one IMU sensors is supported, but "
                 << imuList.size() << " have been attached to the device";
        return false;
    }

    this->imuInterface = imuList[0];

    return true;
}

bool WholeBodyDynamicsDevice::attachAll(const PolyDriverList& p)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    bool ok;
    ok = ok && this->attachAllControlBoard(p);
    ok = ok && this->attachAllVirtualAnalogSensor(p);
    ok = ok && this->attachAllFTs(p);
    ok = ok && this->attachAllIMUs(p);

    if( ok )
    {
        correctlyConfigured = true;
    }

    return ok;
}

double deg2rad(const double angleInDeg)
{
    return angleInDeg*M_PI/180.0;
}

void convertVectorFromDegreesToRadians(iDynTree::VectorDynSize & vector)
{
    for(size_t i=0; i < vector.size(); i++)
    {
        vector(i) = deg2rad(vector(i));
    }

    return;
}


void WholeBodyDynamicsDevice::readSensorsAndUpdateKinematics()
{
    // Read encoders
    sensorReadCorrectly = remappedControlBoardInterfaces.encs->getEncoders(jointPos.data());
    convertVectorFromDegreesToRadians(jointPos);

    // At the moment we are assuming that all joints are revolute

    if( settings.useJointVelocity )
    {
        sensorReadCorrectly = sensorReadCorrectly && remappedControlBoardInterfaces.encs->getEncoderSpeeds(jointVel.data());
        convertVectorFromDegreesToRadians(jointVel);
    }
    else
    {
        jointVel.zero();
    }

    if( settings.useJointAcceleration )
    {
        sensorReadCorrectly = sensorReadCorrectly && remappedControlBoardInterfaces.encs->getEncoderAccelerations(jointAcc.data());
        convertVectorFromDegreesToRadians(jointAcc);
    }
    else
    {
        jointAcc.zero();
    }

    // Read F/T sensors
    for(size_t ft=0; ft < estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
    {
        iDynTree::Wrench bufWrench;
        sensorReadCorrectly = sensorReadCorrectly && ftSensors[ft]->read(ftMeasurement);
        // Format of F/T measurement in YARP/iDynTree is consistent: linear/angular
        iDynTree::toiDynTree(ftMeasurement,bufWrench);
        sensorsMeasurements.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,bufWrench);
    }

    // Read IMU Sensor and update the kinematics in the model
    if( settings.kinematicSource == IMU )
    {
        iDynTree::Vector3 angAcc, linProperAcc, angVel;
        angAcc.zero();
        linProperAcc.zero();
        angVel.zero();
        sensorReadCorrectly = sensorReadCorrectly && imuInterface->read(imuMeasurement);

        // Check format of IMU in YARP http://wiki.icub.org/wiki/Inertial_Sensor
        angVel(0) = deg2rad(imuMeasurement[6]);
        angVel(1) = deg2rad(imuMeasurement[7]);
        angVel(2) = deg2rad(imuMeasurement[8]);

        linProperAcc(0) = imuMeasurement[3];
        linProperAcc(1) = imuMeasurement[4];
        linProperAcc(2) = imuMeasurement[5];

        // Hardcode for the meanwhile
        iDynTree::FrameIndex imuFrameIndex = estimator.model().getFrameIndex("imu_frame");
        estimator.updateKinematicsFromFloatingBase(jointPos,jointVel,jointAcc,imuFrameIndex,linProperAcc,angVel,angAcc);
    }
    else
    {
        iDynTree::Vector3 gravity;

        // this should be valid because it was validated when set
        iDynTree::FrameIndex fixedFrameIndex = estimator.model().getFrameIndex(settings.fixedFrameName);

        gravity(0) = settings.gravity.x;
        gravity(1) = settings.gravity.y;
        gravity(2) = settings.gravity.z;

        estimator.updateKinematicsFromFixedBase(jointPos,jointVel,jointAcc,fixedFrameIndex,gravity);
    }
}

void WholeBodyDynamicsDevice::readContactPoints()
{
    // In this function the location of the external forces acting on the robot
    // are computed. The basic strategy is to assume a contact for each subtree in which the
    // robot is divided by the F/T sensors.

    measuredContactLocations.clear();


}

void addToSummer(iDynTree::Vector6 & buffer, const iDynTree::Wrench & addedWrench)
{
    for(int i=0; i < 6; i++)
    {
        buffer(i) = buffer(i) + addedWrench(i);
    }
}

void computeMean(const iDynTree::Vector6 & buffer, const size_t nrOfSamples, iDynTree::Wrench & mean)
{
    for(int i=0; i < 6; i++)
    {
        mean(i) = buffer(i)/nrOfSamples;
    }
}


void WholeBodyDynamicsDevice::computeCalibration()
{
    if( ongoingCalibration )
    {
        // Run the calibration
        estimator.computeExpectedFTSensorsMeasurements(assumedContactLocationsForCalibration,predictedSensorMeasurementsForCalibration,
                                                       predictedExternalContactWrenchesForCalibration,predictedJointTorquesForCalibration);

        // The kinematics information was already set by the readSensorsAndUpdateKinematics method, just compute the offset and add to the buffer
        for(size_t ft = 0; ft < ftSensors.size(); ft++)
        {
            if( calibratingFTsensor[ft] )
            {
                iDynTree::Wrench estimatedFT;
                iDynTree::Wrench measuredFT;
                predictedSensorMeasurementsForCalibration.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,estimatedFT);
                addToSummer(offsetSumBuffer[ft],measuredFT-estimatedFT);
            }
        }

        // Increase the number of collected samples
        nrOfSamplesUsedUntilNowForCalibration++;

        if( nrOfSamplesUsedUntilNowForCalibration >= nrOfSamplesToUseForCalibration )
        {
            // Compute the offset by averaging the results
            for(size_t ft = 0; ft < ftSensors.size(); ft++)
            {
                if( calibratingFTsensor[ft] )
                {
                    computeMean(offsetSumBuffer[ft],nrOfSamplesUsedUntilNowForCalibration,offset[ft]);
                }
            }

            // We finalize the calibration
            this->endCalibration();
        }
    }

}


void WholeBodyDynamicsDevice::computeExternalForcesAndJointTorques()
{
    // The kinematics information was already set by the readSensorsAndUpdateKinematics method
    estimationWentWell = estimator.estimateExtWrenchesAndJointTorques(measuredContactLocations,sensorsMeasurements,
                                                                       estimateExternalContactWrenches,estimatedJointTorques);
}

void WholeBodyDynamicsDevice::publishEstimatedQuantities()
{
    if( !estimationWentWell )
    {
        yError() << "WholeBodyDynamicsDevice: Error in estimation, no estimates will be published.";
    }
    else
    {
        //Send torques
        publishTorques();

        //Send external contacts
        //publishContacts();

        //Send external wrench estimates
        //publishExternalWrenches();

        //Compute odometry
        //publishOdometry();

        //Send base information to iCubGui
        //publishBaseToGui();

        //Send filtered inertia for gravity compensation
        //publishFilteredInertialForGravityCompensator();

        //Send filtered force torque sensor measurment, if requested
        //publishFilteredFTWithoutOffset();
    }
}

void WholeBodyDynamicsDevice::publishTorques()
{
    for(int axis=0; axis < analogSensorAxes.size(); axis++)
    {
        analogSensorAxes[axis].dev->updateMeasure(analogSensorAxes[axis].localAxis,estimatedJointTorques(axis));
    }
}

void WholeBodyDynamicsDevice::run()
{
    yarp::os::LockGuard guard(this->deviceMutex);

    if( correctlyConfigured )
    {
        // Load settings if modified
        //this->reconfigureClassFromSettings();

        // Read sensor readings
        this->readSensorsAndUpdateKinematics();

        // Compute calibration if we are in calibration mode
        this->computeCalibration();

        // Compute estimated external forces and internal joint torques
        this->computeExternalForcesAndJointTorques();

        // Publish estimated quantities
        this->publishEstimatedQuantities();
    }
}

bool WholeBodyDynamicsDevice::detachAll()
{
    yarp::os::LockGuard guard(this->deviceMutex);

    correctlyConfigured = false;

    return true;
}

bool WholeBodyDynamicsDevice::close()
{
    yarp::os::LockGuard guard(this->deviceMutex);

    correctlyConfigured = false;

    closeRPCPort();
    closeSettingsPort();

    return true;
}

bool WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchOnOneFrame(const std::string & frameName, const int32_t nrOfSamples)
{
    // Let's configure the external forces that then are assume to be active on the robot while calibration

    // Clear the class
    assumedContactLocationsForCalibration.clear();

    // Check if the frame exist
    iDynTree::FrameIndex frameIndex = estimator.model().getFrameIndex(frameName);
    if( frameIndex == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchOnOneFrame impossible to find frame " << frameName << std::endl;
        return false;
    }

    // We assume that the contact is a 6-D the origin of the frame
    iDynTree::UnknownWrenchContact calibrationAssumedContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero());

    bool ok = assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frameIndex,calibrationAssumedContact);

    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchOnOneFrame error for frame " << frameName << std::endl;
        return false;
    }

    setupCalibrationCommonPart(nrOfSamples);

    return true;
}

void WholeBodyDynamicsDevice::setupCalibrationCommonPart(const int32_t nrOfSamples)
{
    nrOfSamplesToUseForCalibration = (size_t)nrOfSamples;
    nrOfSamplesUsedUntilNowForCalibration = 0;

    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        calibratingFTsensor[ft] = true;
    }
    ongoingCalibration = true;

    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        offsetSumBuffer[ft].zero();
    }
}

bool WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchesOnTwoFrames(const std::string & frame1Name, const std::string & frame2Name, const int32_t nrOfSamples)
{
    // Let's configure the external forces that then are assume to be active on the robot while calibration on two links (assumed to be simmetric)

    // Clear the class
    assumedContactLocationsForCalibration.clear();

    // Check if the frame exist
    iDynTree::FrameIndex frame1Index = estimator.model().getFrameIndex(frame1Name);
    if( frame1Index == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchesOnTwoFrames impossible to find frame " << frame1Index << std::endl;
        return false;
    }

    iDynTree::FrameIndex frame2Index = estimator.model().getFrameIndex(frame2Name);
    if( frame2Index == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchesOnTwoFrames impossible to find frame " << frame2Index << std::endl;
        return false;
    }

    // We assume that both  contacts are a 6-D Wrench the origin of the frame
    iDynTree::UnknownWrenchContact calibrationAssumedContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero());

    bool ok = assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frame1Index,calibrationAssumedContact);
    bool ok = ok && assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frame2Index,calibrationAssumedContact);

    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchesOnTwoFrames error" << std::endl;
        return false;
    }

    setupCalibrationCommonPart(nrOfSamples);

    return true;
}

bool WholeBodyDynamicsDevice::calib(const std::string& calib_code, const int32_t nr_of_samples)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yWarning() << "WholeBodyDynamicsDevice::calib ignoring calib_code " << calib_code << std::endl;

    bool ok = this->setupCalibrationWithExternalWrenchOnOneFrame("base_link",nr_of_samples);

    if( !ok )
    {
        return false;
    }

    // Wait for the calibration to finish
    this->waitEndOfCalibration();

    return true;

}

bool WholeBodyDynamicsDevice::calibStanding(const std::string& calib_code, const int32_t nr_of_samples)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yWarning() << "WholeBodyDynamicsDevice::calibStanding ignoring calib_code " << calib_code << std::endl;

    bool ok = this->setupCalibrationWithExternalWrenchesOnTwoFrames("r_sole","l_sole",nr_of_samples);

    if( !ok )
    {
        return false;
    }

    // Wait for the calibration to finish
    this->waitEndOfCalibration();

    return true;

}

bool WholeBodyDynamicsDevice::calibStandingLeftFoot(const std::string& calib_code, const int32_t nr_of_samples)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yWarning() << "WholeBodyDynamicsDevice::calibStandingLeftFoot ignoring calib_code " << calib_code << std::endl;

    bool ok = this->setupCalibrationWithExternalWrenchOnOneFrame("l_sole",nr_of_samples);

    if( !ok )
    {
        return false;
    }


    // Wait for the calibration to finish
    this->waitEndOfCalibration();

    return true;
}

bool WholeBodyDynamicsDevice::calibStandingRightFoot(const std::string& calib_code, const int32_t nr_of_samples)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yWarning() << "WholeBodyDynamicsDevice::calibStandingRightFoot ignoring calib_code " << calib_code << std::endl;

    bool ok = this->setupCalibrationWithExternalWrenchOnOneFrame("r_sole",nr_of_samples);

    if( !ok )
    {
        return false;
    }

    // Wait for the calibration to finish
    this->waitEndOfCalibration();

    return true;

}

bool WholeBodyDynamicsDevice::resetOffset(const std::string& calib_code)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yWarning() << "WholeBodyDynamicsDevice::calib ignoring calib_code " << calib_code << std::endl;

    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        offset[ft].zero();
    }

    return true;
}


bool WholeBodyDynamicsDevice::changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link)
{
    yError() << "WholeBodyDynamicsDevice::changeFixedLinkSimpleLeggedOdometry method not implemented" << std::endl;
    return false;
}


bool WholeBodyDynamicsDevice::resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::__cxx11::string& initial_fixed_link)
{
    yError() << "WholeBodyDynamicsDevice::resetSimpleLeggedOdometry method not implemented" << std::endl;
    return false;
}

bool WholeBodyDynamicsDevice::quit()
{
    yError() << "WholeBodyDynamicsDevice::quit method not implemented" << std::endl;
    return false;
}

size_t WholeBodyDynamicsDevice::getNrOfFTSensors()
{
    return this->offset.size();
}

void WholeBodyDynamicsDevice::waitEndOfCalibration()
{
    // The default value for the semaphore is zero, so this will make us wait until the
    // thread of the device (that is different from the thread handling the RPC) wakes
    calibrationSemaphore.wait();
    return;
}

void WholeBodyDynamicsDevice::endCalibration()
{
    ongoingCalibration = false;
    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        calibratingFTsensor[ft] = false;
    }

    // The default value for the semaphore is zero, so this will make us wait until the
    // thread of the device (that is different from the thread handling the RPC) wakes
    calibrationSemaphore.post();
    return;
}


}



}
}
