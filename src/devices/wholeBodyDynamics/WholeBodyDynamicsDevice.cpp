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

const size_t wholeBodyDynamics_nrOfChannelsOfYARPFTSensor = 6;
const size_t wholeBodyDynamics_nrOfChannelsOfAYARPIMUSensor = 12;

WholeBodyDynamicsDevice::WholeBodyDynamicsDevice(): RateThread(10),
                                                    portPrefix("/wholeBodyDynamics"),
                                                    correctlyConfigured(false),
                                                    sensorReadCorrectly(false),
                                                    estimationWentWell(false),
                                                    validOffsetAvailable(false),
                                                    settingsEditor(settings)
{
    // Calibration quantities
    calibrationBuffers.ongoingCalibration = false;
    calibrationBuffers.calibratingFTsensor.resize(0);
    calibrationBuffers.offsetSumBuffer.resize(0);
    ftProcessors.resize(0);
    calibrationBuffers.estimationSumBuffer.resize(0);
    calibrationBuffers.measurementSumBuffer.resize(0);
    calibrationBuffers.nrOfSamplesToUseForCalibration = 0;
    calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration = 0;

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

bool WholeBodyDynamicsDevice::closeSkinContactListsPorts()
{
    this->portContactsInput.close();
    this->portContactsOutput.close();

    return true;
}

bool WholeBodyDynamicsDevice::closeExternalWrenchesPorts()
{
    for(unsigned int i = 0; i < outputWrenchPorts.size(); i++ )
    {
        if( outputWrenchPorts[i].output_port )
        {
            outputWrenchPorts[i].output_port->close();
            delete outputWrenchPorts[i].output_port;
            outputWrenchPorts[i].output_port = 0;
        }
    }
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
        yError() << "wholeBodyDynamics : open impossible to use the necessary interfaces in remappedControlBoard";
        return ok;
    }

    // Check if the controlboard and the estimator have a consistent number of joints
    int axes = 0;
    remappedControlBoardInterfaces.encs->getAxes(&axes);
    if( axes != (int) estimator.model().getNrOfDOFs() )
    {
        yError() << "wholeBodyDynamics : open estimator model and the remappedControlBoard has an inconsistent number of joints";
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

    yInfo() << "wholeBodyDynamics : Loading model from " << modelFileFullPath;


    ok = estimator.loadModelAndSensorsFromFileWithSpecifiedDOFs(modelFileFullPath,estimationJointNames);
    if( !ok )
    {
        yError() << "wholeBodyDynamics : impossible to create ExtWrenchesAndJointTorquesEstimator from file "
                 << modelFileName << " ( full path: " << modelFileFullPath << " ) ";
        return false;
    }

    if( estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) == 0 )
    {
        yWarning() << "wholeBodyDynamics : the loaded model has 0 FT sensors, so the estimation will use just the model.";
        yWarning() << "wholeBodyDynamics : If you instead want to add the FT sensors to your model, please check iDynTree documentation on how to add sensors to models.";
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
            yError() << "wholeBodyDynamics : openDefaultContactFrames : missing default contact for submodel composed by the links: ";
            const iDynTree::Traversal & subModelTraversal = estimator.submodels().getTraversal(subModelIdx);
            for(iDynTree::TraversalIndex i = 0; i < (iDynTree::TraversalIndex) subModelTraversal.getNrOfVisitedLinks(); i++)
            {
                iDynTree::LinkIndex linkIdx = subModelTraversal.getLink(i)->getIndex();
                yError() << "wholeBodyDynamics : openDefaultContactFrames :" << estimator.model().getLinkName(linkIdx);
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

bool WholeBodyDynamicsDevice::openExternalWrenchesPorts(os::Searchable& config)
{
    // Read ports info from config
    // Load output external wrenches ports informations
    yarp::os::Bottle & output_wrench_bot = config.findGroup("WBD_OUTPUT_EXTERNAL_WRENCH_PORTS");
    if( output_wrench_bot.isNull() )
    {
        // The WBD_OUTPUT_EXTERNAL_WRENCH_PORTS is optional
        return true;
    }

    int nr_of_output_wrench_ports = output_wrench_bot.size() - 1;

    for(int output_wrench_port = 1; output_wrench_port < output_wrench_bot.size(); output_wrench_port++)
    {
        outputWrenchPortInformation wrench_port_struct;
        yarp::os::Bottle *wrench_port = output_wrench_bot.get(output_wrench_port).asList();
        if( wrench_port == NULL || wrench_port->isNull() || wrench_port->size() != 2
            || wrench_port->get(1).asList() == NULL
            || !(wrench_port->get(1).asList()->size() == 2 || wrench_port->get(1).asList()->size() == 3 ) )
        {
            yError() << "wholeBodyDynamics : malformed WBD_OUTPUT_EXTERNAL_WRENCH_PORTS group found in configuration, exiting";
            if( wrench_port )
            {
                yError() << "wholeBodyDynamics : malformed line " << wrench_port->toString();
            }
            else
            {
                yError() << "wholeBodyDynamics : malformed line " << output_wrench_bot.get(output_wrench_port).toString();
                yError() << "wholeBodyDynamics : malformed group " << output_wrench_bot.toString();
            }
            return false;
        }

        wrench_port_struct.port_name = wrench_port->get(0).asString();
        wrench_port_struct.link = wrench_port->get(1).asList()->get(0).asString();

        if( wrench_port->get(1).asList()->size() == 2 )
        {
            // Simple configuration, both the origin and the orientation of the
            // force belong to the same frame
            wrench_port_struct.orientation_frame = wrench_port->get(1).asList()->get(1).asString();
            wrench_port_struct.origin_frame = wrench_port_struct.orientation_frame;
        }
        else
        {
            assert( wrench_port->get(1).asList()->size() == 3 );
            // Complex configuration: the first parameter is the frame of the point of expression,
            // the second parameter is the frame of orientation
            wrench_port_struct.origin_frame = wrench_port->get(1).asList()->get(1).asString();
            wrench_port_struct.orientation_frame = wrench_port->get(1).asList()->get(2).asString();
        }

        outputWrenchPorts.push_back(wrench_port_struct);

    }

    // Load indeces for specified links and frame
    for(unsigned i=0; i < outputWrenchPorts.size(); i++ )
    {
        outputWrenchPorts[i].link_index =
            kinDynComp.getRobotModel().getLinkIndex(outputWrenchPorts[i].link);
        if( outputWrenchPorts[i].link_index < 0 )
        {
            yError() << "wholeBodyDynamics : Link " << outputWrenchPorts[i].link << " not found in the model.";
            return false;
        }

        outputWrenchPorts[i].origin_frame_index =
            kinDynComp.getRobotModel().getFrameIndex(outputWrenchPorts[i].origin_frame);

        if( outputWrenchPorts[i].origin_frame_index < 0 )
        {
            yError() << "wholeBodyDynamics : Frame " << outputWrenchPorts[i].origin_frame << " not found in the model.";
            return false;
        }

        outputWrenchPorts[i].orientation_frame_index =
            kinDynComp.getRobotModel().getFrameIndex(outputWrenchPorts[i].orientation_frame);

        if( outputWrenchPorts[i].orientation_frame_index < 0 )
        {
            yError() << "wholeBodyDynamics : Frame " << outputWrenchPorts[i].orientation_frame_index << " not found in the model.";
            return false;
        }
    }

    // Open ports
    bool ok = true;
    for(unsigned int i = 0; i < outputWrenchPorts.size(); i++ )
    {
        std::string port_name = outputWrenchPorts[i].port_name;
        outputWrenchPorts[i].output_port = new yarp::os::BufferedPort<yarp::sig::Vector>;
        ok = ok && outputWrenchPorts[i].output_port->open(port_name);
        outputWrenchPorts[i].output_vector.resize(wholeBodyDynamics_nrOfChannelsOfYARPFTSensor);
    }

    if( !ok )
    {
        yError() << "wholeBodyDynamics impossible to open port for publishing external wrenches";
        return false;
    }

    return ok;
}


void WholeBodyDynamicsDevice::resizeBuffers()
{
    this->jointPos.resize(estimator.model());
    this->jointVel.resize(estimator.model());
    this->jointAcc.resize(estimator.model());
    this->measuredContactLocations.resize(estimator.model());
    this->ftMeasurement.resize(wholeBodyDynamics_nrOfChannelsOfYARPFTSensor);
    this->imuMeasurement.resize(wholeBodyDynamics_nrOfChannelsOfAYARPIMUSensor);
    this->rawSensorsMeasurements.resize(estimator.sensors());
    this->filteredSensorMeasurements.resize(estimator.sensors());
    this->estimatedJointTorques.resize(estimator.model());
    this->estimateExternalContactWrenches.resize(estimator.model());

    // Resize F/T stuff
    size_t nrOfFTSensors = estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE);
    calibrationBuffers.calibratingFTsensor.resize(nrOfFTSensors,false);
    iDynTree::Wrench zeroWrench;
    zeroWrench.zero();
    calibrationBuffers.offsetSumBuffer.resize(nrOfFTSensors,zeroWrench.asVector());
    calibrationBuffers.measurementSumBuffer.resize(nrOfFTSensors,zeroWrench.asVector());
    calibrationBuffers.estimationSumBuffer.resize(nrOfFTSensors,zeroWrench.asVector());
    calibrationBuffers.assumedContactLocationsForCalibration.resize(estimator.model());
    calibrationBuffers.predictedExternalContactWrenchesForCalibration.resize(estimator.model());
    calibrationBuffers.predictedJointTorquesForCalibration.resize(estimator.model());
    calibrationBuffers.predictedSensorMeasurementsForCalibration.resize(estimator.sensors());

    ftProcessors.resize(nrOfFTSensors);

    // Resize filters
    filters.init(nrOfFTSensors,
                 settings.forceTorqueFilterCutoffInHz,
                 settings.imuFilterCutoffInHz,
                 estimator.model().getNrOfDOFs(),
                 settings.jointVelFilterCutoffInHz,
                 settings.jointAccFilterCutoffInHz,
                 getRate()/1000.0);

    // Resize external wrenches publishing software
    this->netExternalWrenchesExertedByTheEnviroment.resize(estimator.model());
    bool ok = this->kinDynComp.loadRobotModel(estimator.model());

    if( !ok )
    {
        yError() << "wholeBodyDynamics : error in opening KinDynComputation class";
    }
}


bool WholeBodyDynamicsDevice::loadSettingsFromConfig(os::Searchable& config)
{
    // Fill setting with their default values
    settings.kinematicSource             = IMU;
    settings.useJointVelocity            = true;
    settings.useJointAcceleration        = true;
    settings.imuFilterCutoffInHz         = 3.0;
    settings.forceTorqueFilterCutoffInHz = 3.0;
    settings.jointVelFilterCutoffInHz    = 3.0;
    settings.jointAccFilterCutoffInHz    = 3.0;

    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());

    // Check the assumeFixed parameter
    if( prop.check("assume_fixed") )
    {
        if( ! prop.find("assume_fixed").isString() )
        {
            yError() << "wholeBodyDynamics : assume_fixed is present, but it is not a string";
            return false;
        }

        std::string fixedFrameName = prop.find("assume_fixed").asString();

        iDynTree::FrameIndex fixedFrameIndex = estimator.model().getFrameIndex(fixedFrameName);

        if( fixedFrameIndex == iDynTree::FRAME_INVALID_INDEX )
        {
            yError() << "wholeBodyDynamics : assume_fixed is present, but " << fixedFrameName << " is not a frame in the model";
            return false;
        }

        // Add a hardcoded warning, ugly but I think that in the short term is useful
        if( fixedFrameName != "root_link" &&
            fixedFrameName != "l_sole" &&
            fixedFrameName != "r_sole" )
        {
            yWarning() << "wholeBodyDynamics : assume_fixed is set to " << fixedFrameName << " that is not root_link, l_sole or r_sole, so pay attention to correctly set the gravity vector";
        }

        settings.kinematicSource = FIXED_FRAME;
        settings.fixedFrameName = fixedFrameName;
    }


    // fixedFrameGravity is always required even if you
    // use the IMU because the estimation could switch in use a fixed frame
    // via RPC, so we should have a valid gravity to use
    if( prop.check("fixedFrameGravity") &&
        prop.find("fixedFrameGravity").isList() &&
        prop.find("fixedFrameGravity").asList()->size() == 3 )
    {
        settings.fixedFrameGravity.x = prop.find("fixedFrameGravity").asList()->get(0).asDouble();
        settings.fixedFrameGravity.y = prop.find("fixedFrameGravity").asList()->get(1).asDouble();
        settings.fixedFrameGravity.z = prop.find("fixedFrameGravity").asList()->get(2).asDouble();
    }
    else
    {
        yError() << "wholeBodyDynamics : missing required parameter fixedFrameGravity";
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::open(os::Searchable& config)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    bool ok;

    // Load settings in the class
    ok = this->loadSettingsFromConfig(config);
    if( !ok ) return false;

    // Create the estimator
    ok = this->openEstimator(config);
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

    // Open the ports related to publishing external wrenches
    ok = this->openExternalWrenchesPorts(config);


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
                yError() << "wholeBodyDynamics : attachAll error: device "
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
        // A device is considered an ft if it implements IAnalogSensor and has 6 channels
        IAnalogSensor * pAnalogSens = 0;
        if( p[devIdx]->poly->view(pAnalogSens) )
        {
            if( pAnalogSens->getChannels() == (int)wholeBodyDynamics_nrOfChannelsOfYARPFTSensor )
            {
                ftList.push_back(pAnalogSens);
                ftDeviceNames.push_back(p[devIdx]->key);
            }
        }
    }

    if( ftList.size() != estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE) )
    {
        yError() << "wholeBodyDynamicsDevice : was expecting "
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
    std::vector<IAnalogSensor*>  imuList2;

    for(size_t devIdx = 0; devIdx < (size_t)p.size(); devIdx++)
    {
        IGenericSensor * pGenericSensor = 0;
        if( p[devIdx]->poly->view(pGenericSensor) )
        {
            imuList.push_back(pGenericSensor);
        }
    }

    for(size_t devIdx = 0; devIdx < (size_t)p.size(); devIdx++)
    {
        IAnalogSensor * pAnalogSensor = 0;
        if( p[devIdx]->poly->view(pAnalogSensor) )
        {
            int channels = pAnalogSensor->getChannels();

            if( channels == wholeBodyDynamics_nrOfChannelsOfAYARPIMUSensor )
            {
                imuList2.push_back(pAnalogSensor);
            }
        }
    }

    size_t nrOfIMUDetected = imuList.size() + imuList2.size();

    if( nrOfIMUDetected != 1 )
    {
        yError() << "WholeBodyDynamicsDevice was expecting only one IMU, but it did not find " << nrOfIMUDetected << " in the attached devices";
        return false;
    }

    if( imuList.size() == 1 )
    {
        this->imuInterface.useIGenericSensor(imuList[0]);
    }

    if( imuList2.size() == 1 )
    {
        this->imuInterface.useIAnalogSensor(imuList2[0]);
    }


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

void WholeBodyDynamicsDevice::readSensors()
{
    // Read encoders
    sensorReadCorrectly = remappedControlBoardInterfaces.encs->getEncoders(jointPos.data());

    // Convert from degrees (used on wire by YARP) to radians (used by iDynTree)
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

        // Convert from degrees (used on wire by YARP) to radians (used by iDynTree)
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

        // Convert from degrees (used on wire by YARP) to radians (used by iDynTree)
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

        rawSensorsMeasurements.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,bufWrench);
    }

    // Read IMU Sensor
    if( settings.kinematicSource == IMU )
    {
        rawIMUMeasurements.angularAcc.zero();
        rawIMUMeasurements.linProperAcc.zero();
        rawIMUMeasurements.angularVel.zero();

        ok = imuInterface.read(imuMeasurement);

        sensorReadCorrectly = sensorReadCorrectly && ok;

        if( !ok )
        {
            yWarning() << "wholeBodyDynamics warning : imu sensor was not readed correctly";
        }

        // Check format of IMU in YARP http://wiki.icub.org/wiki/Inertial_Sensor
        rawIMUMeasurements.angularVel(0) = deg2rad(imuMeasurement[6]);
        rawIMUMeasurements.angularVel(1) = deg2rad(imuMeasurement[7]);
        rawIMUMeasurements.angularVel(2) = deg2rad(imuMeasurement[8]);

        rawIMUMeasurements.linProperAcc(0) = imuMeasurement[3];
        rawIMUMeasurements.linProperAcc(1) = imuMeasurement[4];
        rawIMUMeasurements.linProperAcc(2) = imuMeasurement[5];
    }
}

void WholeBodyDynamicsDevice::filterSensorsAndRemoveSensorOffsets()
{
    filters.updateCutOffFrequency(settings.forceTorqueFilterCutoffInHz,
                                  settings.imuFilterCutoffInHz,
                                  settings.jointVelFilterCutoffInHz,
                                  settings.jointAccFilterCutoffInHz);

    // Filter and remove offset fromn F/T sensors
    for(size_t ft=0; ft < estimator.sensors().getNrOfSensors(iDynTree::SIX_AXIS_FORCE_TORQUE); ft++ )
    {
        iDynTree::Wrench rawFTMeasure;
        rawSensorsMeasurements.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,rawFTMeasure);

        iDynTree::Wrench rawFTMeasureWithOffsetRemoved  = ftProcessors[ft].filt(rawFTMeasure);

        // Filter the data
        iDynTree::toYarp(rawFTMeasureWithOffsetRemoved,filters.bufferYarp6);

        // Run the filter
        const yarp::sig::Vector & outputFt = filters.forcetorqueFilters[ft]->filt(filters.bufferYarp6);

        iDynTree::Wrench filteredFTMeasure;

        iDynTree::toiDynTree(outputFt,filteredFTMeasure);

        filteredSensorMeasurements.setMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,filteredFTMeasure);
    }

    // Filter joint vel
    if( settings.useJointVelocity )
    {
        iDynTree::toYarp(jointVel,filters.bufferYarpDofs);

        const yarp::sig::Vector & outputJointVel = filters.jntVelFilter->filt(filters.bufferYarpDofs);

        iDynTree::toiDynTree(outputJointVel,jointVel);
    }

    // Filter joint acc
    if( settings.useJointAcceleration )
    {
        iDynTree::toYarp(jointAcc,filters.bufferYarpDofs);

        const yarp::sig::Vector & outputJointAcc = filters.jntAccFilter->filt(filters.bufferYarpDofs);

        iDynTree::toiDynTree(outputJointAcc,jointVel);
    }

    // Filter IMU Sensor
    if( settings.kinematicSource == IMU )
    {
        iDynTree::toYarp(rawIMUMeasurements.linProperAcc,filters.bufferYarp3);

        const yarp::sig::Vector & outputLinAcc = filters.imuLinearAccelerationFilter->filt(filters.bufferYarp3);

        iDynTree::toiDynTree(outputLinAcc,filteredIMUMeasurements.linProperAcc);

        iDynTree::toYarp(rawIMUMeasurements.angularVel,filters.bufferYarp3);

        const yarp::sig::Vector & outputAngVel = filters.imuAngularVelocityFilter->filt(filters.bufferYarp3);

        iDynTree::toiDynTree(outputAngVel,filteredIMUMeasurements.angularVel);

        // For now we just assume that the angular acceleration is zero
        filteredIMUMeasurements.angularAcc.zero();
    }
}

void WholeBodyDynamicsDevice::updateKinematics()
{
    // Read IMU Sensor and update the kinematics in the model
    if( settings.kinematicSource == IMU )
    {
        // Hardcode for the meanwhile
        iDynTree::FrameIndex imuFrameIndex = estimator.model().getFrameIndex("imu_frame");

        estimator.updateKinematicsFromFloatingBase(jointPos,jointVel,jointAcc,imuFrameIndex,
                                                   filteredIMUMeasurements.linProperAcc,filteredIMUMeasurements.angularVel,filteredIMUMeasurements.angularAcc);
    }
    else
    {
        iDynTree::Vector3 gravity;

        // this should be valid because it was validated when set
        iDynTree::FrameIndex fixedFrameIndex = estimator.model().getFrameIndex(settings.fixedFrameName);

        gravity(0) = settings.fixedFrameGravity.x;
        gravity(1) = settings.fixedFrameGravity.y;
        gravity(2) = settings.fixedFrameGravity.z;

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

    for(size_t subModel = 0; subModel < nrOfSubModels; subModel++)
    {
        bool ok = measuredContactLocations.addNewContactInFrame(estimator.model(),
                                                                subModelIndex2DefaultContact[subModel],
                                                               iDynTree::UnknownWrenchContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero()));
        if( !ok )
        {
            yWarning() << "wholeBodyDynamics: Failing in adding default contact for submodel " << subModel;
        }
    }

    // Todo: read contact positions from skin

    return;
}

void addToSummer(iDynTree::Vector6 & buffer, const iDynTree::Wrench & addedWrench)
{
    for(int i=0; i < wholeBodyDynamics_nrOfChannelsOfYARPFTSensor; i++)
    {
        buffer(i) = buffer(i) + addedWrench(i);
    }
}

void computeMean(const iDynTree::Vector6 & buffer, const size_t nrOfSamples, iDynTree::Wrench & mean)
{
    for(int i=0; i < wholeBodyDynamics_nrOfChannelsOfYARPFTSensor; i++)
    {
        mean(i) = buffer(i)/nrOfSamples;
    }
}


void WholeBodyDynamicsDevice::computeCalibration()
{
    if( calibrationBuffers.ongoingCalibration )
    {
        // Todo: Check that the model is actually still during calibration

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
                iDynTree::Wrench measuredRawFT;
                calibrationBuffers.predictedSensorMeasurementsForCalibration.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,estimatedFT);
                rawSensorsMeasurements.getMeasurement(iDynTree::SIX_AXIS_FORCE_TORQUE,ft,measuredRawFT);
                addToSummer(calibrationBuffers.offsetSumBuffer[ft],measuredRawFT-estimatedFT);
                addToSummer(calibrationBuffers.measurementSumBuffer[ft],measuredRawFT);
                addToSummer(calibrationBuffers.estimationSumBuffer[ft],estimatedFT);
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
                    iDynTree::Wrench measurementMean, estimationMean;
                    computeMean(calibrationBuffers.offsetSumBuffer[ft],calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration,ftProcessors[ft].offset());
                    computeMean(calibrationBuffers.measurementSumBuffer[ft],calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration,measurementMean);
                    computeMean(calibrationBuffers.estimationSumBuffer[ft],calibrationBuffers.nrOfSamplesUsedUntilNowForCalibration,estimationMean);

                    yInfo() << "wholeBodyDynamics: Offset for sensor " << estimator.sensors().getSensor(iDynTree::SIX_AXIS_FORCE_TORQUE,ft)->getName() << " " << ftProcessors[ft].offset().toString();
                    yInfo() << "wholeBodyDynamics: obtained assuming a measurement of " << measurementMean.toString() << " and an estimated ft of " << estimationMean.toString();
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
    estimationWentWell = estimator.estimateExtWrenchesAndJointTorques(measuredContactLocations,filteredSensorMeasurements,
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
            publishExternalWrenches();

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
        yError() << "wholeBodyDynamics : publishContacts() error in converting estimated external wrenches from iDynTree to skinDynLib";
    }

    if( ok )
    {
        broadcastData(contactsEstimated,portContactsOutput);
    }
}

void WholeBodyDynamicsDevice::publishExternalWrenches()
{
    if( this->outputWrenchPorts.size() > 0 )
    {
        // Update kinDynComp model
        iDynTree::Vector3 dummyGravity;
        dummyGravity.zero();
        this->kinDynComp.setRobotState(this->jointPos,this->jointVel,dummyGravity);

        // Compute net wrenches for each link
        estimateExternalContactWrenches.computeNetWrenches(netExternalWrenchesExertedByTheEnviroment);
    }


    // Get wrenches from the estimator and publish it on the port
    for(int i=0; i < this->outputWrenchPorts.size(); i++ )
    {
        // Get the wrench in the link frame
        iDynTree::LinkIndex link = this->outputWrenchPorts[i].link_index;
        iDynTree::Wrench & link_f = netExternalWrenchesExertedByTheEnviroment(link);

        // Transform the wrench in the desired frame
        iDynTree::FrameIndex orientation = this->outputWrenchPorts[i].orientation_frame_index;
        iDynTree::FrameIndex origin      = this->outputWrenchPorts[i].origin_frame_index;
        iDynTree::Wrench pub_f = this->kinDynComp.getRelativeTransformExplicit(origin,orientation,link,link)*link_f;

        iDynTree::toYarp(pub_f,outputWrenchPorts[i].output_vector);

        broadcastData<yarp::sig::Vector>(outputWrenchPorts[i].output_vector,
                                         *(outputWrenchPorts[i].output_port));
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
        this->readSensors();

        // Filter sensor and remove offset
        this->filterSensorsAndRemoveSensorOffsets();

        // Update kinematics
        this->updateKinematics();

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
    yarp::os::LockGuard guard(this->deviceMutex);

    correctlyConfigured = false;

    if (isRunning())
        stop();

    closeRPCPort();
    closeSettingsPort();
    closeSkinContactListsPorts();
    closeExternalWrenchesPorts();

    return true;
}

bool WholeBodyDynamicsDevice::close()
{
    // Uncommenting this will result in a deadlock when closing the wbd interface
    //yarp::os::LockGuard guard(this->deviceMutex);

    correctlyConfigured = false;

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
        yError() << "wholeBodyDynamics : setupCalibrationWithExternalWrenchOnOneFrame impossible to find frame " << frameName;
        return false;
    }

    // We assume that the contact is a 6-D the origin of the frame
    iDynTree::UnknownWrenchContact calibrationAssumedContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero());

    bool ok = calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frameIndex,calibrationAssumedContact);

    if( !ok )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithExternalWrenchOnOneFrame error for frame " << frameName;
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
        calibrationBuffers.measurementSumBuffer[ft].zero();
        calibrationBuffers.estimationSumBuffer[ft].zero();
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
        yError() << "wholeBodyDynamics : setupCalibrationWithExternalWrenchesOnTwoFrames impossible to find frame " << frame1Index;
        return false;
    }

    iDynTree::FrameIndex frame2Index = estimator.model().getFrameIndex(frame2Name);
    if( frame2Index == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithExternalWrenchesOnTwoFrames impossible to find frame " << frame2Index;
        return false;
    }

    // We assume that both  contacts are a 6-D Wrench the origin of the frame
    iDynTree::UnknownWrenchContact calibrationAssumedContact(iDynTree::FULL_WRENCH,iDynTree::Position::Zero());

    bool ok = calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frame1Index,calibrationAssumedContact);
    ok = ok && calibrationBuffers.assumedContactLocationsForCalibration.addNewContactInFrame(estimator.model(),frame2Index,calibrationAssumedContact);

    if( !ok )
    {
        yError() << "wholeBodyDynamics : setupCalibrationWithExternalWrenchesOnTwoFrames error";
        return false;
    }

    setupCalibrationCommonPart(nrOfSamples);

    return true;
}

bool WholeBodyDynamicsDevice::calib(const std::string& calib_code, const int32_t nr_of_samples)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yWarning() << "wholeBodyDynamics : calib ignoring calib_code " << calib_code;

    bool ok = this->setupCalibrationWithExternalWrenchOnOneFrame("base_link",nr_of_samples);

    if( !ok )
    {
        return false;
    }

    return true;

}

bool WholeBodyDynamicsDevice::calibStanding(const std::string& calib_code, const int32_t nr_of_samples)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yWarning() << "wholeBodyDynamics : calibStanding ignoring calib_code " << calib_code;

    bool ok = this->setupCalibrationWithExternalWrenchesOnTwoFrames("r_sole","l_sole",nr_of_samples);

    if( !ok )
    {
        return false;
    }

    return true;

}

bool WholeBodyDynamicsDevice::calibStandingLeftFoot(const std::string& calib_code, const int32_t nr_of_samples)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yWarning() << " wholeBodyDynamics : calibStandingLeftFoot ignoring calib_code " << calib_code;

    bool ok = this->setupCalibrationWithExternalWrenchOnOneFrame("l_sole",nr_of_samples);

    if( !ok )
    {
        return false;
    }

    return true;
}

bool WholeBodyDynamicsDevice::calibStandingRightFoot(const std::string& calib_code, const int32_t nr_of_samples)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yWarning() << " wholeBodyDynamics : calibStandingRightFoot ignoring calib_code " << calib_code;

    bool ok = this->setupCalibrationWithExternalWrenchOnOneFrame("r_sole",nr_of_samples);

    if( !ok )
    {
        return false;
    }

    return true;

}

bool WholeBodyDynamicsDevice::resetOffset(const std::string& calib_code)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yWarning() << "wholeBodyDynamics : calib ignoring calib_code " << calib_code;

    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        ftProcessors[ft].offset().zero();
    }

    return true;
}


bool WholeBodyDynamicsDevice::changeFixedLinkSimpleLeggedOdometry(const std::string& /*new_fixed_link*/)
{
    yError() << "wholeBodyDynamics : changeFixedLinkSimpleLeggedOdometry method not implemented";
    return false;
}

double WholeBodyDynamicsDevice::get_forceTorqueFilterCutoffInHz()
{
    yarp::os::LockGuard guard(this->deviceMutex);

    return this->settings.forceTorqueFilterCutoffInHz;
}

bool WholeBodyDynamicsDevice::set_forceTorqueFilterCutoffInHz(const double newCutoff)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    this->settings.forceTorqueFilterCutoffInHz = newCutoff;

    return true;
}

double WholeBodyDynamicsDevice::get_jointVelFilterCutoffInHz()
{
    yarp::os::LockGuard guard(this->deviceMutex);

    return this->settings.jointVelFilterCutoffInHz;
}

bool WholeBodyDynamicsDevice::set_jointVelFilterCutoffInHz(const double newCutoff)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    this->settings.jointVelFilterCutoffInHz = newCutoff;

    return true;
}

double WholeBodyDynamicsDevice::get_jointAccFilterCutoffInHz()
{
    yarp::os::LockGuard guard(this->deviceMutex);

    return this->settings.jointAccFilterCutoffInHz;
}

bool WholeBodyDynamicsDevice::set_jointAccFilterCutoffInHz(const double newCutoff)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    this->settings.jointAccFilterCutoffInHz = newCutoff;

    return true;
}


double WholeBodyDynamicsDevice::get_imuFilterCutoffInHz()
{
    yarp::os::LockGuard guard(this->deviceMutex);

    return this->settings.imuFilterCutoffInHz;
}

bool WholeBodyDynamicsDevice::set_imuFilterCutoffInHz(const double newCutoff)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    this->settings.imuFilterCutoffInHz = newCutoff;

    return true;
}

bool WholeBodyDynamicsDevice::useFixedFrameAsKinematicSource(const std::string& fixedFrame)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    iDynTree::FrameIndex fixedFrameIndex = estimator.model().getFrameIndex(fixedFrame);

    if( fixedFrameIndex == iDynTree::FRAME_INVALID_INDEX )
    {
        yError() << "wholeBodyDynamics : useFixedFrameAsKinematicSource : requested not exiting frame " << fixedFrame << ", method failed";
        return false;
    }

    // Set the kinematic source to a fixed frame
    settings.kinematicSource = FIXED_FRAME;
    settings.fixedFrameName = fixedFrame;

    yInfo() << "wholeBodyDynamics : successfully set the kinematic source to be the fixed frame " << fixedFrame;
    yInfo() << "wholeBodyDynamics : with gravity " << settings.fixedFrameGravity.toString();

    return true;
}

bool WholeBodyDynamicsDevice::useIMUAsKinematicSource()
{
    yarp::os::LockGuard guard(this->deviceMutex);

    yInfo() << "wholeBodyDynamics : successfully set the kinematic source to be the IMU ";

    settings.kinematicSource = IMU;

    return true;
}

bool WholeBodyDynamicsDevice::setUseOfJointVelocities(const bool enable)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    this->settings.useJointVelocity = enable;

    return true;
}

bool WholeBodyDynamicsDevice::setUseOfJointAccelerations(const bool enable)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    this->settings.useJointAcceleration = enable;

    return true;
}

std::string WholeBodyDynamicsDevice::getCurrentSettingsString()
{
   yarp::os::LockGuard guard(this->deviceMutex);

   return settings.toString();
}







bool WholeBodyDynamicsDevice::resetSimpleLeggedOdometry(const std::string& /*initial_world_frame*/, const std::string& /*initial_fixed_link*/)
{
    yError() << " wholeBodyDynamics : resetSimpleLeggedOdometry method not implemented";
    return false;
}

bool WholeBodyDynamicsDevice::quit()
{
    yError() << " wholeBodyDynamics : quit method not implemented";
    return false;
}

size_t WholeBodyDynamicsDevice::getNrOfFTSensors()
{
    return this->ftProcessors.size();
}

void WholeBodyDynamicsDevice::endCalibration()
{
    validOffsetAvailable = true;
    calibrationBuffers.ongoingCalibration = false;
    for(size_t ft = 0; ft < this->getNrOfFTSensors(); ft++)
    {
        calibrationBuffers.calibratingFTsensor[ft] = false;
    }

    yInfo() << " : calibration ended.";

    return;
}

wholeBodyDynamicsDeviceFilters::wholeBodyDynamicsDeviceFilters(): imuLinearAccelerationFilter(0),
                                                                  imuAngularVelocityFilter(0),
                                                                  forcetorqueFilters(0),
                                                                  jntVelFilter(0),
                                                                  jntAccFilter(0),
                                                                  bufferYarp3(0),
                                                                  bufferYarp6(0),
                                                                  bufferYarpDofs(0)
{

}

void wholeBodyDynamicsDeviceFilters::init(int nrOfFTSensors,
                                          double initialCutOffForFTInHz,
                                          double initialCutOffForIMUInHz,
                                          int nrOfDOFsProcessed,
                                          double initialCutOffForJointVelInHz,
                                          double initialCutOffForJointAccInHz,
                                          double periodInSeconds)
{
    // Allocate buffers
    bufferYarp3.resize(3,0.0);
    bufferYarp6.resize(6,0.0);
    bufferYarpDofs.resize(nrOfDOFsProcessed,0.0);

    imuLinearAccelerationFilter =
        new iCub::ctrl::realTime::FirstOrderLowPassFilter(initialCutOffForIMUInHz,periodInSeconds,bufferYarp3);
    imuAngularVelocityFilter =
        new iCub::ctrl::realTime::FirstOrderLowPassFilter(initialCutOffForIMUInHz,periodInSeconds,bufferYarp3);

    forcetorqueFilters.resize(nrOfFTSensors);
    for(int ft_numeric = 0; ft_numeric < nrOfFTSensors; ft_numeric++ )
    {
        forcetorqueFilters[ft_numeric] =
                new iCub::ctrl::realTime::FirstOrderLowPassFilter(initialCutOffForFTInHz,periodInSeconds,bufferYarp6);
    }

    jntVelFilter =
        new iCub::ctrl::realTime::FirstOrderLowPassFilter(initialCutOffForJointVelInHz,periodInSeconds,bufferYarpDofs);
    jntAccFilter =
        new iCub::ctrl::realTime::FirstOrderLowPassFilter(initialCutOffForJointAccInHz,periodInSeconds,bufferYarpDofs);
}


void wholeBodyDynamicsDeviceFilters::updateCutOffFrequency(double cutoffForFTInHz,
                                                           double cutOffForIMUInHz,
                                                           double cutOffForJointVelInHz,
                                                           double cutOffForJointAccInHz)
{
    imuLinearAccelerationFilter->setCutFrequency(cutOffForIMUInHz);
    imuAngularVelocityFilter->setCutFrequency(cutOffForIMUInHz);

    for(size_t ft_numeric = 0; ft_numeric < forcetorqueFilters.size(); ft_numeric++ )
    {
        forcetorqueFilters[ft_numeric]->setCutFrequency(cutoffForFTInHz);
    }

    jntVelFilter->setCutFrequency(cutOffForJointVelInHz);
    jntAccFilter->setCutFrequency(cutOffForJointAccInHz);
}

void wholeBodyDynamicsDeviceFilters::fini()
{
    if( imuLinearAccelerationFilter )
    {
        delete imuLinearAccelerationFilter;
        imuLinearAccelerationFilter = 0;
    }

    if( imuAngularVelocityFilter )
    {
        delete imuAngularVelocityFilter;
        imuAngularVelocityFilter = 0;
    }

    for(int ft_numeric = 0; ft_numeric < forcetorqueFilters.size(); ft_numeric++ )
    {
        delete forcetorqueFilters[ft_numeric];
        forcetorqueFilters[ft_numeric] = 0;
    }

    forcetorqueFilters.resize(0);

    if( jntVelFilter )
    {
        delete jntVelFilter;
        jntVelFilter = 0;
    }

    if( jntAccFilter )
    {
        delete jntAccFilter;
        jntAccFilter = 0;
    }
}

wholeBodyDynamicsDeviceFilters::~wholeBodyDynamicsDeviceFilters()
{
    fini();
}


IGenericSensorEmulator::IGenericSensorEmulator(): m_genericSensor(0),
                                                  m_analogSensor(0)
{
}

IGenericSensorEmulator::~IGenericSensorEmulator()
{

}

void IGenericSensorEmulator::useIAnalogSensor(IAnalogSensor* _analogSensor)
{
    m_analogSensor = _analogSensor;
    m_genericSensor = 0;
}

void IGenericSensorEmulator::useIGenericSensor(IGenericSensor* _genericSensor)
{
    m_genericSensor = _genericSensor;
    m_analogSensor  = 0;
}

bool IGenericSensorEmulator::read(sig::Vector& out)
{
    if( m_genericSensor )
    {
        return m_genericSensor->read(out);
    }

    if( m_analogSensor )
    {
        return (m_analogSensor->read(out) == IAnalogSensor::AS_OK);
    }

    return false;
}

bool IGenericSensorEmulator::calibrate(int ch, double v)
{
    if( m_genericSensor )
    {
        return m_genericSensor->calibrate(ch,v);
    }

    if( m_analogSensor )
    {
        return (m_analogSensor->calibrateChannel(ch,v)  == IAnalogSensor::AS_OK);
    }

    return false;
}

bool IGenericSensorEmulator::getChannels(int* nc)
{
    if( m_genericSensor )
    {
        return m_genericSensor->getChannels(nc);
    }

    if( m_analogSensor )
    {
        *nc = m_analogSensor->getChannels();
        return true;
    }

    return false;
}









}
}


