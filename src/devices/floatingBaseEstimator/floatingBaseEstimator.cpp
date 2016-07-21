#include "floatingBaseEstimator.h"

#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

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

const double floatingBaseEstimator_sensorTimeoutInSeconds = 2.0;

floatingBaseEstimator::floatingBaseEstimator(): RateThread(10),
                                                portPrefix("/floatingBaseEstimator"),
                                                correctlyConfigured(false),
                                                sensorReadCorrectly(false),
                                                estimationWentWell(false)
{
}

floatingBaseEstimator::~floatingBaseEstimator()
{

}


bool floatingBaseEstimator::openPorts()
{
    this->floatingBaseEstimator_IDLServer::yarp().attachAsServer(rpcPort);

    bool ok = rpcPort.open(portPrefix+"/rpc");

    if( !ok )
    {
        yError() << "floatingBaseEstimator: Impossible to open port " << portPrefix+"/rpc";
        return false;
    }

    ok = iCubGuiPort.open(portPrefix+"/base:o");
    if( !ok )
    {
        yError() << "floatingBaseEstimator: Impossible to open port " << portPrefix+"/base:o";
        return false;
    }

    ok = WBIPort.open(portPrefix+"/floatingbasestate:o");
    if( !ok )
    {
        yError() << "floatingBaseEstimator: Impossible to open port " << portPrefix+"/floatingbasestate:o";
        return false;
    }

    return true;
}

bool floatingBaseEstimator::closePorts()
{
    rpcPort.close();
    iCubGuiPort.close();
    WBIPort.close();

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


bool floatingBaseEstimator::openRemapperControlBoard(os::Searchable& config)
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
        yError() << "floatingBaseEstimator : open impossible to use the necessary interfaces in remappedControlBoard";
        return ok;
    }

    // Check if the controlboard and the estimator have a consistent number of joints
    int axes = 0;
    remappedControlBoardInterfaces.encs->getAxes(&axes);
    if( axes != (int) estimator.model().getNrOfDOFs() )
    {
        yError() << "floatingBaseEstimator : open estimator model and the remappedControlBoard has an inconsistent number of joints";
        return false;
    }

    return true;
}


bool floatingBaseEstimator::openEstimator(os::Searchable& config)
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

    yInfo() << "floatingBaseEstimator : Loading model from " << modelFileFullPath;

    ok = estimator.loadModelFromFileWithSpecifiedDOFs(modelFileFullPath,estimationJointNames);
    if( !ok )
    {
        yError() << "floatingBaseEstimator : impossible to create SimpleLeggedOdometry from file "
                 << modelFileName << " ( full path: " << modelFileFullPath << " ) ";
        return false;
    }

    this->resizeBuffers();
    return true;
}


void floatingBaseEstimator::resizeBuffers()
{
    this->jointPos.resize(estimator.model());
    this->jointVelSetToZero.resize(estimator.model());

    this->homMatrixBuffer.resize(4,4);
}


bool floatingBaseEstimator::loadSettingsFromConfig(os::Searchable& config)
{
    yarp::os::Property prop;
    prop.fromString(config.toString().c_str());

    if( prop.check("initialFixedFrame") &&
        prop.find("initialFixedFrame").isString() )
    {
        initialFixedFrame = prop.find("initialFixedFrame").asString();
    }
    else
    {
        yError() << "floatingBaseEstimator : missing required parameter initialFixedFrame";
        return false;
    }

    if( prop.check("initialWorldFrame") &&
        prop.find("initialWorldFrame").isString() )
    {
         initialWorldFrame = prop.find("initialWorldFrame").asString();
    }
    else
    {
        initialWorldFrame = initialFixedFrame;
    }

    return true;
}

bool floatingBaseEstimator::open(os::Searchable& config)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    bool ok;

    // Load settings in the class
    ok = this->loadSettingsFromConfig(config);
    if( !ok ) return false;

    // Create the estimator
    ok = this->openEstimator(config);
    if( !ok ) return false;

    // Open ports
    ok = this->openPorts();
    if( !ok ) return false;

    // Open the controlboard remapper
    ok = this->openRemapperControlBoard(config);
    if( !ok ) return false;

    return true;
}

bool floatingBaseEstimator::attachAllControlBoard(const PolyDriverList& p)
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
        yError() << " floatingBaseEstimator::attachAll in attachAll of the remappedControlBoard";
        return false;
    }

    return true;
}

bool floatingBaseEstimator::attachAll(const PolyDriverList& p)
{
    yarp::os::LockGuard guard(this->deviceMutex);

    bool ok = true;
    ok = ok && this->attachAllControlBoard(p);

    if( ok )
    {
        this->start();
    }

    return ok;
}

double floatingBaseEstimator_deg2rad(const double angleInDeg)
{
    return angleInDeg*M_PI/180.0;
}

void floatingBaseEstimator_convertVectorFromDegreesToRadians(iDynTree::VectorDynSize & vector)
{
    for(size_t i=0; i < vector.size(); i++)
    {
        vector(i) = floatingBaseEstimator_deg2rad(vector(i));
    }

    return;
}

void floatingBaseEstimator::readSensors()
{
    // Read encoders
    sensorReadCorrectly = remappedControlBoardInterfaces.encs->getEncoders(jointPos.data());

    // Convert from degrees (used on wire by YARP) to radians (used by iDynTree)
    floatingBaseEstimator_convertVectorFromDegreesToRadians(jointPos);

    jointVelSetToZero.zero();
}

void floatingBaseEstimator::updateKinematics()
{
    estimationWentWell = estimator.updateKinematics(jointPos);
}

void floatingBaseEstimator::publishEstimatedQuantities()
{
    if( !estimationWentWell )
    {
        yError() << "floatingBaseEstimator : Error in estimation, no estimates will be published.";
    }
    else
    {
        publishFloatingBasePosInWBIFormat();
        publishFloatingBasePosIniCubGuiFormat();
    }
}

void floatingBaseEstimator::publishFloatingBasePosIniCubGuiFormat()
{
    // TODO this is quite an hack, and should be moved to a better place
    // (a portmonitor on the port?)

    iDynTree::Transform world_H_base
        = this->estimator.getWorldLinkTransform(this->estimator.model().getDefaultBaseLink());

    double roll,pitch,yaw;

    world_H_base.getRotation().getRPY(roll,pitch,yaw);

    yarp::sig::Vector & iCubGuiVector = this->iCubGuiPort.prepare();
    iCubGuiVector.resize(6);

    const double RAD2DEG = 180.0/(M_PI);

    iCubGuiVector[0] = RAD2DEG*roll;
    iCubGuiVector[1] = RAD2DEG*pitch;
    iCubGuiVector[2] = RAD2DEG*yaw;

    //Set linear part (iCubGui wants the root offset in millimeters)
    const double METERS2MILLIMETERS = 1000.0;
    iCubGuiVector[3] = METERS2MILLIMETERS*world_H_base.getPosition()(0);
    iCubGuiVector[4] = METERS2MILLIMETERS*world_H_base.getPosition()(1);
    iCubGuiVector[5] = METERS2MILLIMETERS*world_H_base.getPosition()(2);

    //Add offset to avoid lower forces to be hided by the floor
    iCubGuiVector[5] = iCubGuiVector[5] + 1000.0;

    this->iCubGuiPort.write();

}

void floatingBaseEstimator::publishFloatingBasePosInWBIFormat()
{
    iDynTree::Transform world_H_base
        = this->estimator.getWorldLinkTransform(this->estimator.model().getDefaultBaseLink());

    iDynTree::toYarp(world_H_base.asHomogeneousTransform(),this->homMatrixBuffer);

    yarp::os::Bottle & bot = this->WBIPort.prepare();
    bot.clear();
    bot.addList().read(this->homMatrixBuffer);

    WBIPort.write();
}



template <class T> void fbe_broadcastData(T& _values, yarp::os::BufferedPort<T>& _port)
{
    if (_port.getOutputCount()>0 )
    {
        _port.prepare()  = _values ;
        _port.write();
    }
}


void floatingBaseEstimator::run()
{
    yarp::os::LockGuard guard(this->deviceMutex);

    // Read sensor readings
    this->readSensors();

    if( sensorReadCorrectly )
    {
        if( !correctlyConfigured )
        {
            // first run, configure the estimator
            this->updateKinematics();
            correctlyConfigured = this->estimator.init(initialFixedFrame,initialWorldFrame);
        }

        if( correctlyConfigured )
        {
            // Update kinematics
            this->updateKinematics();

            // Publish estimated quantities
            this->publishEstimatedQuantities();
        }
    }
    else
    {
        yError() << "floatingBaseEstimator : Error in sensors readings, no estimates will be published.";
    }
}

bool floatingBaseEstimator::detachAll()
{
    yarp::os::LockGuard guard(this->deviceMutex);

    correctlyConfigured = false;

    if (isRunning())
    {
        stop();
    }

    this->remappedControlBoardInterfaces.multwrap->detachAll();

    closePorts();

    return true;
}

bool floatingBaseEstimator::close()
{
    this->remappedControlBoard.close();

    correctlyConfigured = false;

    return true;
}

//////////////////////////
/// RPC methods
/////////////////////////

bool floatingBaseEstimator::resetSimpleLeggedOdometry(const std::string& initial_world_frame,
                                                              const std::string& initial_fixed_frame)
{
    yarp::os::LockGuard guard(this->deviceMutex);
    return this->estimator.init(initial_fixed_frame,initial_world_frame);
}

bool floatingBaseEstimator::changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_frame)
{
    yarp::os::LockGuard guard(this->deviceMutex);
    return this->estimator.changeFixedFrame(new_fixed_frame);
}

std::string floatingBaseEstimator::getCurrentSettingsString()
{
    yarp::os::LockGuard guard(this->deviceMutex);
    std::stringstream ss;
    ss << "Current settings for floatingBaseEstimator\n";
    ss << "Used estimator: simpleLeggedOdometry\n";
    ss << "Current fixedLink: " << this->estimator.getCurrentFixedLink() << "\n";
    ss << "Current world_H_fixedLink: " << this->estimator.getWorldLinkTransform(this->estimator.model().getLinkIndex(this->estimator.getCurrentFixedLink())).toString() << "\n";
    return ss.str();
}

}

}

