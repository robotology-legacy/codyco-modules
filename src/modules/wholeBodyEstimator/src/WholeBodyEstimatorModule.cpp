#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include "WholeBodyEstimatorModule.h"
#include "WholeBodyEstimatorThread.h"

using namespace yarp::os;
using namespace yarpWbi;

WholeBodyEstimatorModule::WholeBodyEstimatorModule()
{
    m_period = 10;
}

bool WholeBodyEstimatorModule::configure(ResourceFinder &rf) 
{
    
    if (!rf.check("module_parameters"))
    {
        yError("Group module_parameters was not specified in the configuration file of this module. Please fix it and try again.");
        return false;
    } else
    {
        // Fill m_module_params with module parameters
        m_module_params.fromString(rf.findGroup("module_parameters").tail().toString());
        yInfo(" [WholeBodyEstimatorModule::configure] module_parameters group contents are: %s ", m_module_params.toString().c_str());
        
        m_period = m_module_params.find("period").asInt();
        m_module_name = m_module_params.find("name").asString();
    }
    
    std::string wbiConfFile;
    yarp::os::Property yarpWbiOptions;
    wbi::IDList RobotDynamicModelJoints;
    
    if (!rf.check("wbi_conf_file"))
    {
        yError("WBI configuration file name not specified in config file of this module.");
        return false;
    } else
    {
        wbiConfFile = rf.findFile("wbi_conf_file");
        yarpWbiOptions.fromConfigFile(wbiConfFile);
    }
    
    
    // Configure yarpWholeBodySensors before passing it to WholeBodyEstimatorThread
    // Get model joints list
    std::string modelJointsListName = rf.check("joints_list",
                                                yarp::os::Value("ROBOT_DYNAMIC_MODEL_JOINTS"),
                                                "Name of the list of joint used for the current robot").asString().c_str();
    if( !loadIdListFromConfig(modelJointsListName,rf,RobotDynamicModelJoints) )
    {
        if( !loadIdListFromConfig(modelJointsListName,yarpWbiOptions,RobotDynamicModelJoints) )
        {
            yError("[wholeBodyEstimatorModule::configure] Impossible to load wbiId joint list with name %s\n",modelJointsListName.c_str());
            return false;
        }
    }
    
    yarpWholeBodySensors* wbs = new yarpWholeBodySensors(m_module_name.c_str(), yarpWbiOptions);

    // Adding encoders
    wbs->addSensors(wbi::SENSOR_ENCODER, RobotDynamicModelJoints);
    //TODO: Accelerometer and gyroscopes should be added here
    //wbi->addSensors(wbi::SENSOR_ACCELEROMETER, enabledAccelerometersList);
    //wbi->addSensors(wbi::SENSOR_GYROSCOPES, enabledGyroscopesList);
    
    // Initializing sensor interface
    if(!wbs->init())
    {
        yError("[wholeBodyEstimatorModule::configure] Error while initializing whole body estimator interface.Closing module");
        return false;
    } else
    {
        yInfo("[wholeBodyEstimatorModule::configure] Whole Body Sensors initialized correctly.");
    }

    
    m_estimatorThread = new WholeBodyEstimatorThread(rf, wbs, m_period);
    if (!m_estimatorThread->start())
    {
        yError("[WholeBodyEstimatorModule::configure] Couldn't start thread!");
        return false;
    }

    return true;
}

bool WholeBodyEstimatorModule::updateModule()
{
    bool ret = true;
    return ret;
}

bool WholeBodyEstimatorModule::close()
{
    yDebug("Closing module...");
    if (m_estimatorThread)
    {
        m_estimatorThread->stop();
        delete m_estimatorThread;
        m_estimatorThread = NULL;
    }
    return true;
}