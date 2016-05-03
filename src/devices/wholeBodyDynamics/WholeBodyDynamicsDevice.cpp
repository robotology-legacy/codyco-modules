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
                                                    validOffsetAvailable(false),
                                                    imuInterface(0),
                                                    settingsEditor(settings),
                                                    calibrationSemaphore(0)
{
    // Calibration quantities
    calibrationBuffers.ongoingCalibration = false;
    calibrationBuffers.calibratingFTsensor.resize(0);
    calibrationBuffers.offsetSumBuffer.resize(0);
    calibrationBuffers.offset.resize(0);
    calibrationBuffers.nrOfSamplesToUseForCalibration = 0;
    calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration = 0;

}

WholeBodyDynamicsDevice::~WholeBodyDynamicsDevice()
{

}


bool WholeBodyDynamicsDevice::openSettingsPort()
{
    // Fill setting with their default values
    settings.kinematicSource = IMU;
    settings.useJointVelocity = 0;
    settings.useJointAcceleration = 0;

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

void addVectorOfStringToProperty(yarp::os::Property& prop, std::string key, std::vector<std::string> & list)
{
    prop.addGroup(key);
    yarp::os::Bottle & bot = prop.findGroup(key).addList();
    for(size_t i=0; i < list.size(); i++)
    {
        bot.addString(list[i].c_str());
    }
    return;
}

bool getUsedDOFsList(os::Searchable& config, std::vector<std::string> & usedDOFs)
{
    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());

    yarp::os::Bottle *propAxesNames=prop.find("axesNames").asList();
    if(propAxesNames==0)
    {
       yError() <<"WholeBodyDynamicsDevice: Error parsing parameters: \"axesNames\" should be followed by a list\n";
       return false;
    }

    usedDOFs.resize(propAxesNames->size());
    for(int ax=0; ax < propAxesNames->size(); ax++)
    {
        usedDOFs[ax] = propAxesNames->get(ax).asString().c_str();
    }

    return true;
}


bool WholeBodyDynamicsDevice::openRemapperControlBoard(os::Searchable& config)
{
    // Pass to the remapper just the relevant parameters (axesList)
    yarp::os::Property propRemapper;
    propRemapper.put("device","controlboardremapper");
    bool ok = getUsedDOFsList(config,estimationJointNames);
    if(!ok) return false;

    addVectorOfStringToProperty(propRemapper,"axesNames",estimationJointNames);

    ok = remappedControlBoard.open(propRemapper);

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
    // get the list of considered dofs from config
    bool ok = getUsedDOFsList(config,estimationJointNames);
    if(!ok) return false;

    yarp::os::ResourceFinder & rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    std::string modelFileName = "model.urdf";
    if( config.check("modelFile") && config.find("modelFile").isString() )
    {
        modelFileName = config.find("modelFile").asString();
    }

    std::string modelFileFullPath = rf.findFileByName(modelFileName);
    ok = estimator.loadModelAndSensorsFromFileWithSpecifiedDOFs(modelFileFullPath,estimationJointNames);
    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice::open impossible to create ExtWrenchesAndJointTorquesEstimator from file "
                 << modelFileName << " ( full path: " << modelFileFullPath << " ) ";
        return false;
    }

    this->resizeBuffers();
    return true;
}

bool WholeBodyDynamicsDevice::openDefaultContactFrames(os::Searchable& /*config*/)
{
    // For now we hardcode this info
    defaultContactFrames.push_back("l_hand");
    defaultContactFrames.push_back("r_hand");
    defaultContactFrames.push_back("root_link");
    defaultContactFrames.push_back("l_lower_leg");
    defaultContactFrames.push_back("r_lower_leg");
    defaultContactFrames.push_back("l_sole");
    defaultContactFrames.push_back("r_sole");
    defaultContactFrames.push_back("l_elbow_1");
    defaultContactFrames.push_back("r_elbow_1");

    // We build the defaultContactFramesIdx vector
    std::vector<iDynTree::FrameIndex> defaultContactFramesIdx;
    defaultContactFramesIdx.clear();
    for(size_t i=0; i < defaultContactFrames.size(); i++)
    {
        iDynTree::FrameIndex idx = estimator.model().getFrameIndex(defaultContactFrames[i]);

        if( idx == iDynTree::FRAME_INVALID_INDEX )
        {
            yWarning() << "Frame " << defaultContactFrames[i] << " not found in the model, discarding it";
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

    // We indicate with FRAME_INVALID_INDEX the fact that we still don't have a default contact for the given submodel
    subModelIndex2DefaultContact.resize(nrOfSubModels,iDynTree::FRAME_INVALID_INDEX);

    for(size_t i=0; i < defaultContactFramesIdx.size(); i++)
    {
        size_t subModelIdx = estimator.submodels().getSubModelOfFrame(estimator.model(),defaultContactFramesIdx[i]);

        // If the subModel of the frame still does not have a default contact, we add it
        if( subModelIndex2DefaultContact[subModelIdx] == iDynTree::FRAME_INVALID_INDEX )
        {
            subModelIndex2DefaultContact[subModelIdx] = defaultContactFramesIdx[i];
        }
    }


    // Let's check that every submodel has a default contact position
    for(size_t subModelIdx = 0; subModelIdx < nrOfSubModels; subModelIdx++)
    {
        if( subModelIndex2DefaultContact[subModelIdx] == iDynTree::FRAME_INVALID_INDEX )
        {
            yError() << "WholeBodyDynamicsDevice::openDefaultContactFrames : missing default contact for submodel composed by the links: ";
            const iDynTree::Traversal & subModelTraversal = estimator.submodels().getTraversal(subModelIdx);
            for(iDynTree::TraversalIndex i = 0; i < (iDynTree::TraversalIndex) subModelTraversal.getNrOfVisitedLinks(); i++)
            {
                iDynTree::LinkIndex linkIdx = subModelTraversal.getLink(i)->getIndex();
                yError() << "WholeBodyDynamicsDevice::openDefaultContactFrames :" << estimator.model().getLinkName(linkIdx);
            }

            return false;
        }
    }

    return true;
}

bool WholeBodyDynamicsDevice::openSkinContactListPorts(os::Searchable& config)
{
    bool ok = this->portContactsInput.open(portPrefix+"/skin_contacts:i");
    if(!ok) return ok;

    ok = this->portContactsOutput.open(portPrefix+"/contacts:o");
    if(!ok) return ok;

    // Configure the conversion helper to read/publish data on this ports
    yarp::os::Bottle & bot = config.findGroup("IDYNTREE_SKINDYNLIB_LINKS");
    for(int i=1; i < bot.size(); i++ )
    {
        yarp::os::Bottle * map_bot = bot.get(i).asList();
        if( map_bot->size() != 2 || map_bot->get(1).asList() == NULL ||
            map_bot->get(1).asList()->size() != 3 )
        {
            yError() << "WholeBodyDynamicsDevice: IDYNTREE_SKINDYNLIB_LINKS group is malformed (" << map_bot->toString() << ")";
            return false;
        }

        std::string iDynTree_link_name = map_bot->get(0).asString();
        std::string iDynTree_skinFrame_name = map_bot->get(1).asList()->get(0).asString();
        int skinDynLib_body_part = map_bot->get(1).asList()->get(1).asInt();
        int skinDynLib_link_index = map_bot->get(1).asList()->get(2).asInt();

        bool ret_sdl = conversionHelper.addSkinDynLibAlias(estimator.model(),
                                                           iDynTree_link_name,iDynTree_skinFrame_name,
                                                           skinDynLib_body_part,skinDynLib_link_index);

        if( !ret_sdl )
        {
            yError() << "WholeBodyDynamicsDevice: IDYNTREE_SKINDYNLIB_LINKS link " << iDynTree_link_name
                      << " and frame " << iDynTree_skinFrame_name << " and not found in urdf model";
            return false;
        }
    }

    return ok;
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
    calibrationBuffers.calibratingFTsensor.resize(nrOfFTSensors,false);
    iDynTree::Wrench zeroWrench;
    zeroWrench.zero();
    calibrationBuffers.offset.resize(nrOfFTSensors,zeroWrench);
    calibrationBuffers.offsetSumBuffer.resize(nrOfFTSensors,zeroWrench.asVector());
    calibrationBuffers.assumedContactLocationsForCalibration.resize(estimator.model());
    calibrationBuffers.predictedExternalContactWrenchesForCalibration.resize(estimator.model());
    calibrationBuffers.predictedJointTorquesForCalibration.resize(estimator.model());
    calibrationBuffers.predictedSensorMeasurementsForCalibration.resize(estimator.sensors());
}


bool WholeBodyDynamicsDevice::loadSettingsFromConfig(os::Searchable& /*config*/)
{
    return true;
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

    // Open rpc port
    ok = this->openRPCPort();
    if( !ok ) return false;

    // Open settings port
    ok = this->openSettingsPort();
    if( !ok ) return false;

    // Open the controlboard remapper
    ok = this->openRemapperControlBoard(config);
    if( !ok ) return false;

    ok = this->openDefaultContactFrames(config);
    if( !ok ) return false;

    // Open the skin-related ports
    ok = this->openSkinContactListPorts(config);
    if( !ok ) return false;


    return true;
}

bool WholeBodyDynamicsDevice::attachAllControlBoard(const PolyDriverList& p)
{
    PolyDriverList controlBoardList;
    for(size_t devIdx = 0; devIdx < (size_t) p.size(); devIdx++)
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
    for(size_t devIdx = 0; devIdx < (size_t)p.size(); devIdx++)
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
    std::vector<std::string>     ftDeviceNames;
    for(size_t devIdx = 0; devIdx < (size_t)p.size(); devIdx++)
    {
        IAnalogSensor * pAnalogSens = 0;
        if( p[devIdx]->poly->view(pAnalogSens) )
        {
            ftList.push_back(pAnalogSens);
            ftDeviceNames.push_back(p[devIdx]->key);
        }
    }

    if( ftList.size() != estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) )
    {
        yError() << " WholeBodyDynamicsDevice was expecting "
                 << estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE)
                 << " from the model, but got " << ftList.size() << " FT sensor in the attach list.";
        return false;
    }

    // For now we assume that the name of the F/T sensor device match the sensor name in the URDF
    // In the future we could use a new fancy sensor interface
    ftSensors.resize(ftList.size());
    for(size_t IDTsensIdx=0; IDTsensIdx < ftSensors.size(); IDTsensIdx++)
    {
        std::string sensorName = estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,IDTsensIdx)->getName();

        // Search for a suitable device
        int deviceThatHasTheSameNameOfTheSensor = -1;
        for(size_t deviceIdx = 0; deviceIdx < ftList.size(); deviceIdx++)
        {
            if( ftDeviceNames[deviceIdx] == sensorName )
            {
                deviceThatHasTheSameNameOfTheSensor = deviceIdx;
            }
        }

        if( deviceThatHasTheSameNameOfTheSensor == -1 )
        {
            yError() << "WholeBodyDynamicsDevice was expecting a sensor named " << sensorName << " but it did not find one in the attached devices";
            return false;
        }

        ftSensors[IDTsensIdx] = ftList[deviceThatHasTheSameNameOfTheSensor];
    }

    return true;
}


bool WholeBodyDynamicsDevice::attachAllIMUs(const PolyDriverList& p)
{
    std::vector<IGenericSensor*> imuList;

    for(size_t devIdx = 0; devIdx < (size_t)p.size(); devIdx++)
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

    bool ok = true;
    ok = ok && this->attachAllControlBoard(p);
    ok = ok && this->attachAllVirtualAnalogSensor(p);
    ok = ok && this->attachAllFTs(p);
    ok = ok && this->attachAllIMUs(p);

    ok = this->setupCalibrationWithExternalWrenchOnOneFrame("base_link",100);

    if( ok )
    {
        correctlyConfigured = true;
        yDebug() << " WholeBodyDynamicsDevice correctly configured, starting the thread and the calibration";
        this->start();
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
    bool ok;

    if( !sensorReadCorrectly )
    {
        yWarning() << "wholeBodyDynamics warning : joint positions was not readed correctly";
    }

    // At the moment we are assuming that all joints are revolute

    if( settings.useJointVelocity )
    {
        ok = remappedControlBoardInterfaces.encs->getEncoderSpeeds(jointVel.data());
        sensorReadCorrectly = sensorReadCorrectly && ok;
        if( !ok )
        {
            yWarning() << "wholeBodyDynamics warning : joint velocities was not readed correctly";
        }

        convertVectorFromDegreesToRadians(jointVel);
    }
    else
    {
        jointVel.zero();
    }

    if( settings.useJointAcceleration )
    {
        ok = remappedControlBoardInterfaces.encs->getEncoderAccelerations(jointAcc.data());
        sensorReadCorrectly = sensorReadCorrectly && ok;
        if( !ok )
        {
            yWarning() << "wholeBodyDynamics warning : joint accelerations was not readed correctly";
        }
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
        int ftRetVal = ftSensors[ft]->read(ftMeasurement);

        ok = (ftRetVal == IAnalogSensor::AS_OK);

        sensorReadCorrectly = sensorReadCorrectly && ok;

        if( !ok )
        {
            std::string sensorName = estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName();
            yWarning() << "wholeBodyDynamics warning : FT sensor " << sensorName << " was not readed correctly";
        }

        // Format of F/T measurement in YARP/iDynTree is consistent: linear/angular
        iDynTree::toiDynTree(ftMeasurement,bufWrench);
        // Remove offset
        bufWrench = bufWrench - calibrationBuffers.offset[ft];
        sensorsMeasurements.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,bufWrench);
    }

    // Read IMU Sensor and update the kinematics in the model
    if( settings.kinematicSource == IMU )
    {
        iDynTree::Vector3 angAcc, linProperAcc, angVel;
        angAcc.zero();
        linProperAcc.zero();
        angVel.zero();
        ok = imuInterface->read(imuMeasurement);
        sensorReadCorrectly = sensorReadCorrectly && ok;

        if( !ok )
        {
            yWarning() << "wholeBodyDynamics warning : imu sensor was not readed correctly";
        }

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

    // For now just put the default contact points
    size_t nrOfSubModels = estimator.submodels().getNrOfSubModels();

    //yDebug() << "WholeBodyDynamicsDevice nrOfSubModels " << nrOfSubModels;

    for(size_t subModel = 0; subModel < nrOfSubModels; subModel++)
    {
        bool ok = measuredContactLocations.addNewContactInFrame(estimator.model(),
                                                                subModelIndex2DefaultContact[subModel],
                                                               iDynTree::UnknownWrenchContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero()));

        /*
        yDebug() << "WholeBodyDynamicsDevice for submodel " << subModel;
        yDebug() << "WholeBodyDynamicsDevice adding contact in frame " << estimator.model().getFrameName(subModelIndex2DefaultContact[subModel]);
        yDebug() << "WholeBodyDynamicsDevice of link " << estimator.model().getLinkName(estimator.model().getFrameLink(subModelIndex2DefaultContact[subModel]));
        yDebug() << "WholeBodyDynamicsDevice total nr of contacts : " << measuredContactLocations.getNrOfContactsForLink(estimator.model().getFrameLink(subModelIndex2DefaultContact[subModel]));
        */

        if( !ok )
        {
            yWarning() << "wholeBodyDynamics: Failing in adding default contact for submodel " << subModel;
        }
    }

    // Todo: read contact positions from skin
    //yDebug() << "WholeBodyDynamicsDevice measuredContactLocations " << measuredContactLocations.toString(estimator.model());

    return;
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
    if( calibrationBuffers.ongoingCalibration )
    {
        // Run the calibration
        estimator.computeExpectedFTSensorsMeasurements(calibrationBuffers.assumedContactLocationsForCalibration,
                                                       calibrationBuffers.predictedSensorMeasurementsForCalibration,
                                                       calibrationBuffers.predictedExternalContactWrenchesForCalibration,
                                                       calibrationBuffers.predictedJointTorquesForCalibration);

        // The kinematics information was already set by the readSensorsAndUpdateKinematics method, just compute the offset and add to the buffer
        for(size_t ft = 0; ft < ftSensors.size(); ft++)
        {
            if( calibrationBuffers.calibratingFTsensor[ft] )
            {
                iDynTree::Wrench estimatedFT;
                iDynTree::Wrench measuredFT;
                calibrationBuffers.predictedSensorMeasurementsForCalibration.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,estimatedFT);
                sensorsMeasurements.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,measuredFT);
                addToSummer(calibrationBuffers.offsetSumBuffer[ft],measuredFT-estimatedFT);
            }
        }

        // Increase the number of collected samples
        calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration++;

        if( calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration >= calibrationBuffers.nrOfSamplesToUseForCalibration )
        {
            // Compute the offset by averaging the results
            for(size_t ft = 0; ft < ftSensors.size(); ft++)
            {
                if( calibrationBuffers.calibratingFTsensor[ft] )
                {
                    computeMean(calibrationBuffers.offsetSumBuffer[ft],calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration,calibrationBuffers.offset[ft]);

                    yInfo() << "wholeBodyDynamics: Offset for sensor " << estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName() << " " << calibrationBuffers.offset[ft].toString();
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
    //yDebug() << "WholeBodyDynamicsDevice::computeExternalForcesAndJointTorques() " << measuredContactLocations.toString(estimator.model());
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
        // Only send estimation if a valid offset is available
        if( validOffsetAvailable )
        {

            //Send torques
            publishTorques();

            //Send external contacts
            publishContacts();

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
}

template <class T> void broadcastData(T& _values, yarp::os::BufferedPort<T>& _port)
{
    if (_port.getOutputCount()>0 )
    {
        _port.prepare()  = _values ;
        _port.write();
    }
}

void WholeBodyDynamicsDevice::publishTorques()
{

    for(size_t axis=0; axis < remappedVirtualAnalogSensorAxis.size(); axis++)
    {
        if( remappedVirtualAnalogSensorAxis[axis].dev )
        {
            remappedVirtualAnalogSensorAxis[axis].dev->updateMeasure(remappedVirtualAnalogSensorAxis[axis].localAxis,estimatedJointTorques(axis));
        }
    }
}

void WholeBodyDynamicsDevice::publishContacts()
{
    // Clear the buffer of published forces
    contactsEstimated.clear();

    // Convert the result of estimation
    bool ok = conversionHelper.updateSkinContactListFromLinkContactWrenches(estimator.model(),estimateExternalContactWrenches,contactsEstimated);

    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice::publishContacts() error in converting estimated external wrenches from iDynTree to skinDynLib";
    }

    if( ok )
    {
        //yDebug() << "WholeBodyDynamicsDevice::publishContacts " << estimateExternalContactWrenches.toString(estimator.model());
        //yDebug() << "WholeBodyDynamicsDevice::publishContacts converted to " << contactsEstimated.toString();
        broadcastData(contactsEstimated,portContactsOutput);
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

        // Read contacts info from the skin or from assume contact location
        this->readContactPoints();

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
    yDebug() << " WholeBodyDynamicsDevice detachAll called ";
    yarp::os::LockGuard guard(this->deviceMutex);

    correctlyConfigured = false;

    yDebug() << " WholeBodyDynamicsDevice detachAll, closing thread ";
    if (isRunning())
        stop();

    closeRPCPort();
    closeSettingsPort();

    yDebug() << " WholeBodyDynamicsDevice detachAll returning";

    return true;
}

bool WholeBodyDynamicsDevice::close()
{
    yDebug() << " WholeBodyDynamicsDevice close called ";

    // Uncommenting this will result in a deadlock when closing the wbd interface
    //yarp::os::LockGuard guard(this->deviceMutex);

    correctlyConfigured = false;

    yDebug() << " WholeBodyDynamicsDevice close returning ";


    return true;
}

bool WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchOnOneFrame(const std::string & frameName, const int32_t nrOfSamples)
{
    // Let's configure the external forces that then are assume to be active on the robot while calibration

    // Clear the class
    calibrationBuffers.assumedContactLocationsForCalibration.clear();

    // Check if the frame exist
    iDynTree::FrameIndex frameIndex = estimator.model().getFrameIndex(frameName);
    if( frameIndex == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchOnOneFrame impossible to find frame " << frameName;
        return false;
    }

    // We assume that the contact is a 6-D the origin of the frame
    iDynTree::UnknownWrenchContact calibrationAssumedContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero());

    bool ok = calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frameIndex,calibrationAssumedContact);

    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchOnOneFrame error for frame " << frameName;
        return false;
    }

    setupCalibrationCommonPart(nrOfSamples);

    return true;
}

void WholeBodyDynamicsDevice::setupCalibrationCommonPart(const int32_t nrOfSamples)
{
    calibrationBuffers.nrOfSamplesToUseForCalibration = (size_t)nrOfSamples;
    calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration = 0;

    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        calibrationBuffers.calibratingFTsensor[ft] = true;
    }
    calibrationBuffers.ongoingCalibration = true;

    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        calibrationBuffers.offsetSumBuffer[ft].zero();
    }
}

bool WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchesOnTwoFrames(const std::string & frame1Name, const std::string & frame2Name, const int32_t nrOfSamples)
{
    // Let's configure the external forces that then are assume to be active on the robot while calibration on two links (assumed to be simmetric)

    // Clear the class
    calibrationBuffers.assumedContactLocationsForCalibration.clear();

    // Check if the frame exist
    iDynTree::FrameIndex frame1Index = estimator.model().getFrameIndex(frame1Name);
    if( frame1Index == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchesOnTwoFrames impossible to find frame " << frame1Index;
        return false;
    }

    iDynTree::FrameIndex frame2Index = estimator.model().getFrameIndex(frame2Name);
    if( frame2Index == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchesOnTwoFrames impossible to find frame " << frame2Index;
        return false;
    }

    // We assume that both  contacts are a 6-D Wrench the origin of the frame
    iDynTree::UnknownWrenchContact calibrationAssumedContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero());

    bool ok = calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frame1Index,calibrationAssumedContact);
    ok = ok && calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frame2Index,calibrationAssumedContact);

    if( !ok )
    {
        yError() << "WholeBodyDynamicsDevice::setupCalibrationWithExternalWrenchesOnTwoFrames error";
        return false;
    }

    setupCalibrationCommonPart(nrOfSamples);

    return true;
}

bool WholeBodyDynamicsDevice::calib(const std::string& calib_code, const int32_t nr_of_samples)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yWarning() << "WholeBodyDynamicsDevice::calib ignoring calib_code " << calib_code;

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

    yWarning() << "WholeBodyDynamicsDevice::calibStanding ignoring calib_code " << calib_code;

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

    yWarning() << "WholeBodyDynamicsDevice::calibStandingLeftFoot ignoring calib_code " << calib_code;

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

    yWarning() << "WholeBodyDynamicsDevice::calibStandingRightFoot ignoring calib_code " << calib_code;

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

    yWarning() << "WholeBodyDynamicsDevice::calib ignoring calib_code " << calib_code;

    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        calibrationBuffers.offset[ft].zero();
    }

    return true;
}


bool WholeBodyDynamicsDevice::changeFixedLinkSimpleLeggedOdometry(const std::string& /*new_fixed_link*/)
{
    yError() << "WholeBodyDynamicsDevice::changeFixedLinkSimpleLeggedOdometry method not implemented";
    return false;
}


bool WholeBodyDynamicsDevice::resetSimpleLeggedOdometry(const std::string& /*initial_world_frame*/, const std::string& /*initial_fixed_link*/)
{
    yError() << "WholeBodyDynamicsDevice::resetSimpleLeggedOdometry method not implemented";
    return false;
}

bool WholeBodyDynamicsDevice::quit()
{
    yError() << "WholeBodyDynamicsDevice::quit method not implemented";
    return false;
}

size_t WholeBodyDynamicsDevice::getNrOfFTSensors()
{
    return this->calibrationBuffers.offset.size();
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
    validOffsetAvailable = true;
    calibrationBuffers.ongoingCalibration = false;
    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        calibrationBuffers.calibratingFTsensor[ft] = false;
    }

    // The default value for the semaphore is zero, so this will make us wait until the
    // thread of the device (that is different from the thread handling the RPC) wakes
    calibrationSemaphore.post();

    yInfo() << "WholeBodyDynamicsDevice: calibration ended.";

    return;
}


}
}

